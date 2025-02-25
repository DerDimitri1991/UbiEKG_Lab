#include <arduinoFFT.h>
#include "BluetoothSerial.h"         // Include the Bluetooth Serial library to enable Bluetooth communication.
#include <Adafruit_NeoPixel.h>       // Include the Adafruit NeoPixel library for controlling RGB LEDs.
#include <esp_sleep.h>               // Include Library for deep sleep.
#include "driver/gpio.h"             // ESP32 library for low-level GPIO control (pin configuration, input/output settings, etc.)
#include <Wire.h>                    // Arduino I2C library for communicating with I2C devices (sensors, displays, etc.)
#include <ds3231.h>                  // Include Library for time
#include <LittleFS.h>                // Include Library for file Storage


// Bluetooth Device Name
String device_name = "UbiEKG";       // Define the name of the Bluetooth device.

// Pin Definitions
#define BUTTON_PIN 38                // Onboard Button foe deep sleep.
#define ECG_PIN A0                   // Analog input pin for the ECG signal.
#define SAMPLE_RATE 200              // Sampling rate in Hz (samples per second).
#define WINDOW_SIZE 30               // Size of the moving average window for signal integration.
#define LoPlus 32                    // Set GPIO 32 as input for LO+ (leads-off detection).
#define LoMinus 14                   // Set GPIO 14 as input for LO- (leads-off detection).
#define Power_LED 13                 // Onboard LED to indicate Board is running.
#define FFT_SIZE 256                 // Number of samples for FFT (must be a power of 2) and min 256.
#define VBATPIN A13                  // Pin for reading Battery capacity
#define VBAT_MIN 3.0                 // Minimum voltage for the battery (adjust to your battery specs)
#define VBAT_MAX 4.2                // Maximum voltage for the battery (fully charged)
#define BATTERY_SMOOTHING 10        // Number of samples for smoothing

// DS3231 clock
struct ts t;
bool rtcUpdated = false; // Flag to check if RTC is updated
bool isCommandProcessed = false;

// FFT and HRV Variables
float vReal[FFT_SIZE];               // Real part of FFT input.
float vImag[FFT_SIZE];               // Imaginary part of FFT input.
ArduinoFFT<float> FFT(vReal, vImag, FFT_SIZE, SAMPLE_RATE);
float rrBuffer[FFT_SIZE];            // Buffer to store RR intervals.
int rrBufferIndex = 0;               // Index for RR buffer
bool bufferReady = false;           // Flag for FFT processing readiness
int fftProgress = 0;                 // Current FFT progress counter
int totalFFTSteps = FFT_SIZE;        // Total steps for FFT acquisition
bool showFFTProgress = true;         // Flag to control the display of the FFT progress bar
int garbageSkipCount = 10;          // Number of initial RR intervals to skip
int garbageSkipped = 0;             // Counter for skipped RR intervals

// Heartbeat and RMSSD Calculation Variables
int sample_size = 20;                // For BPM, SDNN, and RMSSD calculations.
static float rrIntervals[20];        // Store last 20 RR intervals for HRV.
static int rrIndex = 0;              // Index for RR interval buffer
float successiveDiffSum = 0;         // To keep track of the sum of squared differences.
int rrCount = 0;                     // Counter for the number of RR intervals considered.
const int totalRRCount = 600;        // Total RR intervals for 5-minute calculation.
float finalRMSSD = 0;                // Store the last calculated RMSSD value

// Signal Detection and Threshold Variables
unsigned long lastPeakTime = 0;      // Stores the timestamp of the last detected peak (R-wave).
unsigned long currentPeakTime = 0;   // Stores the timestamp of the current peak (R-wave).
unsigned long lastQRS = 0;           // Time of the last detected QRS complex.
float signalMax = 0;                 // Maximum signal value for adaptive thresholding.
float signalMin = 0;                 // Minimum signal value for adaptive thresholding.
float threshold = 0;                 // Adaptive threshold.

// Flags and Counters
bool Warmup = false;                 // For warm-up phase to discard garbage values.
int skipRRCount = 0;                 // Counter to skip the first 20 RR intervals.
const int skipThreshold = 40;        // Number of intervals to skip.
bool isWarmupComplete = false;       // Flag to track if the warmup phase is complete.

// Output Variables
float bpm = 0;                       // Variable to store calculated Beats Per Minute (BPM).
int rateAlert = 0;                   // Indicates Tachycardia, Bradycardia, and Normal heart rate over Bluetooth Serial.
String Stress = "Aquiring data";       // Current stress level
String StressProgress = "Initializing";  // Current progress of FFT/HRV operations
String RMSSDProgress = "Initializing";   // Progress of RMSSD calculation
String StressValue = "";                // String representation of stress level.

// Bluetooth Communication
BluetoothSerial SerialBT;            // Create a Bluetooth Serial object.

unsigned long lastLogTime = 0; // Time of the last logged entry

// Function Prototypes
void resetVariables();
float derivativeFilter(float input);
float square(float input);
float movingWindowIntegration(float input);
bool detectQRS(float input);
void processrrInterval(float rrInterval);
void CalculateBPM(float rrIntervall);
void DetectdHeartCondition(float bpm);
float calculateRMSSD(float newRR, float previousRR);
void addRRInterval(float rrInterval);
void computeFFT();
void analyzeHRV();
float calculateBandPower(float* spectrum, int start, int end);

void setup() {
    Serial.begin(9600);              // Initialize serial communication at 9600 baud.
    pinMode(LoPlus, INPUT);          // Set GPIO 32 as input for LO+ (leads-off detection).
    pinMode(LoMinus, INPUT);         // Set GPIO 14 as input for LO- (leads-off detection).
    SerialBT.begin(device_name);     // Start Bluetooth with the defined device name.
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Set the button pin as input.
    pinMode(Power_LED, OUTPUT);

    // Initialize RTC
    Wire.begin();              // Initialize I2C on ESP32's default pins.
    DS3231_init(DS3231_CONTROL_INTCN); // Initialize DS3231.
    Serial.println("RTC initialized. Waiting for time sync via Bluetooth.");

    // Print initial RTC time
    DS3231_get(&t);
    Serial.printf("Initial Time: %02d:%02d:%02d, Date: %02d/%02d/%04d\n",
                  t.hour, t.min, t.sec, t.mday, t.mon, t.year);

if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed. Formatting...");
    if (LittleFS.format()) {
        Serial.println("LittleFS formatted successfully.");
        if (!LittleFS.begin()) {
            Serial.println("Failed to mount LittleFS after formatting.");
            return;
        }
    } else {
        Serial.println("LittleFS formatting failed.");
        return;
    }
} else {
    Serial.println("LittleFS mounted successfully.");
}


    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

    Serial.printf("Device \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
    // Print a message indicating that the device is ready to pair via Bluetooth.

        // Check wake-up cause
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("Woke up from deep sleep!");
        digitalWrite(Power_LED, HIGH);
    } else {
        Serial.println("Powering on normally.");
        digitalWrite(Power_LED, LOW);
    }

    // Set button as wake-up source
    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(BUTTON_PIN), 0);
}

void loop() {
    //////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////// Power LED shows that device is running //////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////
    digitalWrite(Power_LED, HIGH); // Power LED
    ////////////////////////////////////////////////////////////////////////////////////////////
    // Check button press to enter deep sleep
    ///////////////////////////////////////////////////////////////////////////////////////////
if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("Button pressed! Preparing for deep sleep...");
    enterDeepSleep();
    delay(100); // Debounce delay
}

    // Check for Bluetooth commands
    if (SerialBT.available()) {
        String command = SerialBT.readStringUntil('\n'); // Read the incoming command
        Serial.println("Received command: " + command); // Debugging output
        handleBluetoothCommand(command); // Process the command
    }

// Check for USB Serial commands
if (Serial.available()) {
    String usbCommand = Serial.readStringUntil('\n'); // Read the incoming command
    Serial.println("USB Command: " + usbCommand); // Debug output for received command

    if (usbCommand == "DELETE_FILE") {
        deleteFile("/stress_log.csv"); // Delete the specified file
    } 
    else if (usbCommand == "LIST_FILE") {
        listFiles(); // List all files on LittleFS
    } 
    else if (usbCommand.startsWith("APPEND_LOG")) {
        int spaceIndex = usbCommand.indexOf(' '); // Find the space separating command and data
        if (spaceIndex != -1) {
            String data = usbCommand.substring(spaceIndex + 1); // Extract the data after the space
            appendToFile("/stress_log.csv", data);
            Serial.println("Data appended to log: " + data);
        } else {
            Serial.println("ERROR: No data provided with APPEND_LOG");
        }
    } 
    else {
        Serial.println("Unknown command received over USB.");
    }
}



    // Print current RTC time periodically
  /*  
    if (millis() % 1000 < 10) {
        DS3231_get(&t);
        Serial.printf("Current Time: %02d:%02d:%02d, Date: %02d/%02d/%04d\n",
                      t.hour, t.min, t.sec, t.mday, t.mon, t.year);
    }
*/

    ////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Code for Heart Detection /////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////
    // Check if leads are off (LO+ or LO- is high).
    if ((digitalRead(LoPlus) == 1) || (digitalRead(LoMinus) == 1)) {
        float batteryPercent = getBatteryPercentage();
        String csvValues = String("Disconnected") + "," + String(batteryPercent, 1) + "%";
        SerialBT.println(csvValues);  // Send the new format over Bluetooth.
        //Serial.println(csvValues);  // Send the new format over Bluetooth.
            // Print initial RTC time
        resetVariables();
        delay(100);
    } else {
        ///////////////////////////////////////////////////////////////////////////////
        ////////////////// Apply Pan–Tompkins Algorithm //////////////////////////////
        /////////////////////////////////////////////////////////////////////////////
        int rawSignal = analogRead(ECG_PIN);                          // Read raw ECG signal.
        float differentiatedSignal = derivativeFilter(rawSignal);     // Compute derivative.
        float squaredSignal = square(differentiatedSignal);           // Square the signal.
        float integratedSignal = movingWindowIntegration(squaredSignal); // Integrate the signal.

        /////////////////////////////////////////////////////////////////////////
        ////////////////////// Adaptive Threshold Update ///////////////////////
        ///////////////////////////////////////////////////////////////////////
        signalMax = max(signalMax, integratedSignal); // Tracks the highest value (peak).
        signalMin = min(signalMin, integratedSignal); // Tracks the lowest value (trough).
        threshold = 0.5 * (signalMax - signalMin) + signalMin; // Dynamically calculates a threshold.

        ////////////////////////////////////////////////////////////////////////////
        ////////////////// Detect Heartbeat Per Minute ////////////////////////////
        ///////////////////////////////////////////////////////////////////////////
        if (detectQRS(integratedSignal)) {                            // Detect QRS complex.
            currentPeakTime = millis();                               // Record the timestamp of the peak.
            if (lastPeakTime > 0) {                                   // If there was a previous peak:
                unsigned long rrInterval = currentPeakTime - lastPeakTime;
                addRRInterval((rrInterval/1000.0));                    // Konvert RR interval in seconds and add to FFT buffer
                processrrInterval(rrInterval);                        // Process RR interval.
                CalculateBPM(rrInterval);                             // Calculate BPM.
                DetectdHeartCondition(bpm);                           // Detect heart condition.
                float batteryPercent = getBatteryPercentage();

                int previousIndex = (rrIndex - 1 + sample_size) % sample_size;
                float rmssd = calculateRMSSD(rrIntervals[(previousIndex - 1 + sample_size) % sample_size],
                                             rrIntervals[previousIndex]); // Calculate RMSSD.

                // Update RMSSD display
                String rmssdDisplay = Warmup && finalRMSSD > 0 ? String(finalRMSSD, 2) : "Aquiring data";                                             

                if (Stress.startsWith("High")) {
                    // Log high-stress events to file
                    handleStressDetection(bpm, rmssd, StressValue);
                    Serial.println("High stress detected. Data logged.");
                }

                // Send values over Bluetooth
                String csvValues = String(rrInterval) + "," + String(bpm) + "," + String(rmssdDisplay) + "," +
                   Stress + "," + StressProgress + "," + RMSSDProgress + "," + String(rateAlert)+ "," + String(batteryPercent, 1) + "%";
                SerialBT.println(csvValues);
                //Serial.println(csvValues);


            }
            lastPeakTime = currentPeakTime;
            lastQRS = currentPeakTime;

            signalMax = 0; // Reset signal bounds after each detected QRS.
            signalMin = 0;

            if (bufferReady) {
                computeFFT();
                analyzeHRV();
            }
        }
    }

    delay(1000 / SAMPLE_RATE); // Wait for the next sample based on the sampling rate.
}
//////////////////////////////////////////////////
// Function to enter deep sleep
/////////////////////////////////////////////////
void enterDeepSleep() {
    SerialBT.end();       // Stop Bluetooth communication
    Serial.flush();       // Ensure all data is sent
    detachInterrupt(digitalPinToInterrupt(LoPlus));
    detachInterrupt(digitalPinToInterrupt(LoMinus));

    pinMode(LoPlus, INPUT_PULLUP);
    pinMode(LoMinus, INPUT_PULLUP);
    digitalWrite(Power_LED, LOW);

    delay(100);
    esp_deep_sleep_start();
}
/////////////////////////////////////////////////////////
// Function to handle Bluetooth commands
////////////////////////////////////////////////////////
void handleBluetoothCommand(String command) {
    Serial.println("Debug: Entering handleBluetoothCommand");
    Serial.println("Debug: Received command: " + command);
    
    if (command.startsWith("SET_TIME")) {
        // Parse the incoming command
        int params[6];
        int index = 0;
        int lastIndex = command.indexOf(','); // Start from the first comma

        for (int i = 0; i < 6; i++) {
            int nextIndex = command.indexOf(',', lastIndex + 1); // Find the next comma
            params[i] = command.substring(lastIndex + 1, nextIndex).toInt(); // Extract the value
            lastIndex = nextIndex; // Update the last index
        }

        // Map parsed values to the RTC time
        t.hour = params[0];            // Hour
        t.min = params[1];             // Minute
        t.sec = params[2];             // Second
        t.mday = params[3];            // Day
        t.mon = params[4];             // Month (already 1-based)
        t.year = params[5];     // Year in DS3231 format (e.g., 2025 -> 25)

        // Set the RTC
        DS3231_set(t);

        // Debug output
        Serial.printf("RTC Set to: %02d:%02d:%02d, %02d/%02d/%04d\n",
                      t.hour, t.min, t.sec, t.mday, t.mon, t.year);

        // Confirm RTC update
        SerialBT.println("RTC updated successfully!");
        Serial.println("RTC updated successfully!");
    } else if (command == "SEND_FILE") {
        sendFileOverBluetooth("/stress_log.csv");

    } else {
        Serial.println("Unknown command received.");
    }




}
//////////////////////////////////////////////////////
//Debugging
//Add Custom Data for debugging
/////////////////////////////////////////////////////
void appendToFile(const char* fileName, String data) {
    File file = LittleFS.open(fileName, FILE_APPEND);
    if (!file) {
        Serial.printf("Failed to open %s for appending.\n", fileName);
        return;
    }

    file.println(data); // Append the data as a new line
    file.close();
    Serial.printf("Appended data to %s: %s\n", fileName, data.c_str());
}
////////////////////////////////////////////////////////
//Debugging
// show safed files
///////////////////////////////////////////////////////
void listFiles() {
    Serial.println("Listing files:");
    File root = LittleFS.open("/");
    if (!root) {
        Serial.println("Failed to open root directory.");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        Serial.printf("File: %s, Size: %d bytes\n", file.name(), file.size());

        // Read and display the contents of each file
        Serial.printf("Contents of %s:\n", file.name());
        while (file.available()) {
            Serial.write(file.read());
        }
        Serial.println("\n--- End of File ---");

        file = root.openNextFile();
    }
    root.close();
}

////////////////////////////////////////////////////////
// Function to delete file contents
////////////////////////////////////////////////////////
void deleteFile(const char* fileName) {
    File file = LittleFS.open(fileName, FILE_WRITE); // Open the file in write mode
    if (!file) {
        Serial.printf("Failed to open %s for deletion.\n", fileName);
        return;
    }

    file.close(); // Closing the file immediately truncates it
    Serial.printf("File %s deleted successfully.\n", fileName);
}
// Function to send file content over Bluetooth with progress
void sendFileOverBluetooth(const char* fileName) {
    File file = LittleFS.open(fileName, FILE_READ);
    if (!file) {
        SerialBT.println("ERROR: File not found");
        Serial.println("ERROR: File not found");
        return;
    }

    // Get the total file size
    size_t totalSize = file.size();
    size_t bytesSent = 0;

    SerialBT.println("FILE_START");
    Serial.println("FILE_START");

    unsigned long lastSendTime = 0;   // Tracks the last time a line was sent
    const unsigned long sendInterval = 100; // 50 ms delay between lines

    while (file.available()) {
        // Check if enough time has passed since the last line was sent
        if (millis() - lastSendTime >= sendInterval) {
            // Read a line from the file
            String line = file.readStringUntil('\n');

            // Update the bytes sent count
            bytesSent += line.length() + 1; // Include newline character

            // Calculate progress percentage
            int progress = (bytesSent * 100) / totalSize;

            // Append the progress percentage to the line
            line += "," + String(progress);

            // Send the line with progress over Bluetooth and Serial
            SerialBT.println(line);
            Serial.println(line);

            // Update the last send time
            lastSendTime = millis();
        }
    }

    file.close();
    SerialBT.println("FILE_END");
    Serial.println("File transfer complete.");
}

//////////////////////////////////////////////////////////
//Function to calculate battery Charge
/////////////////////////////////////////////////////////
float getBatteryPercentage() {
    static float batteryReadings[BATTERY_SMOOTHING] = {0}; // Circular buffer for battery readings
    static int index = 0; // Current index in the buffer
    static float sum = 0; // Sum of the buffer values

    // Read the current battery voltage
    float measuredvbat = analogReadMilliVolts(VBATPIN) * 2 / 1000.0; // Convert to volts
    float batteryPercent = ((measuredvbat - VBAT_MIN) / (VBAT_MAX - VBAT_MIN)) * 100.0;
    batteryPercent = constrain(batteryPercent, 0, 100); // Ensure percentage stays between 0% and 100%

    // Update the moving average buffer
    sum -= batteryReadings[index];       // Subtract the oldest value from the sum
    batteryReadings[index] = batteryPercent; // Replace it with the new reading
    sum += batteryReadings[index];       // Add the new value to the sum

    index = (index + 1) % BATTERY_SMOOTHING; // Move to the next index (circular buffer)

    // Calculate the smoothed percentage
    float smoothedPercent = sum / BATTERY_SMOOTHING;

    // Cast to int to remove the decimal part
    return static_cast<int>(smoothedPercent); 
}
//////////////////////////////////////////////////////////////
// Function to reset variables (included for clarity)
//////////////////////////////////////////////////////////////
void resetVariables() {
    // Reset critical variables to prevent crashes or incorrect calculations
    // Reset rrIntervals buffer
    for (int i = 0; i < sample_size; i++) {
        rrIntervals[i] = 0;
    }

    // Reset counters and flags
    skipRRCount = 0;
    rrIndex = 0;
    rrCount = 0;
    rrBufferIndex = 0;
    bufferReady = false;
    Warmup = false;

    // Reset adaptive thresholding variables
    signalMax = 0;
    signalMin = 0;
    threshold = 0;

    // Reset peak time tracking
    lastPeakTime = 0;
    currentPeakTime = 0;

    // Reset heart rate and HRV metrics
    bpm = 0;
    rateAlert = 0;
    successiveDiffSum = 0;

    // Clear rrBuffer
    memset(rrBuffer, 0, sizeof(rrBuffer));

    // Reset stress level
    Stress = "Aquiring data";
    fftProgress = true;
}

///////////////////////////////////////////////////////////////////////////
///////////////////////implement derivative filtering//////////////////////
///////////////////////////////////////////////////////////////////////////
float derivativeFilter(float input) {
    static float x[5] = {0, 0, 0, 0, 0}; // Circular buffer to store the last 5 input samples.

    // Shift the previous values in the buffer
    x[4] = x[3];
    x[3] = x[2];
    x[2] = x[1];
    x[1] = x[0];
    x[0] = input;

    // Apply the 5-point derivative filter
    float derivative = 0.1 * (-x[4] - 2 * x[3] + 2 * x[1] + x[0]);

    return derivative;
}
///////////////////////////////////////////////////////////////////////////
///////////////////////implement squaring of the signal/////////////////
//////////////////////////////////////////////////////////////////////////
float square(float input) {
    return input * input;  // Square the input value.
}

///////////////////////////////////////////////////////////////////////////
///////////////////////implement moving Window Integration/////////////////
//////////////////////////////////////////////////////////////////////////
//This movingWindowIntegration at a small, 
//fixed-sized "window" of recent input values (determined by WINDOW_SIZE) and calculates their average. 
//The term "window" refers to this subset of values that "moves" forward as new data comes in.
float movingWindowIntegration(float input) {
    static float buffer[WINDOW_SIZE] = {0};  // Buffer to store samples.
    static int index = 0;                    // Current buffer index.
    static float sum = 0;                    // Sum of buffer values.

    sum -= buffer[index];        // Subtract oldest value.
    buffer[index] = input;       // Store new input.
    sum += buffer[index];        // Add new value to sum.

    index = (index + 1) % WINDOW_SIZE;  // Update index (wrap around).

    return sum / WINDOW_SIZE;  // Return average over the window.
}


///////////////////////////////////////////////////////////////////////////
///////////////////////implement Peak detection/////////////////
//////////////////////////////////////////////////////////////////////////
bool detectQRS(float input) {
    static unsigned long lastPeakTime = 0;
    if (input > threshold) {                        //If input surpasses this threshold, it might be a QRS complex.
        unsigned long currentTime = millis();
        if (currentTime - lastPeakTime > 200) {     //This condition ensures that at least 200 milliseconds have passed since the last detected QRS complex. 
                                                    //Corresponds to a maximum heart rate of 300 BPM (1000 ms ÷ 200 ms)
                                                    //Prevents detecting multiple peaks from the same QRS complex (caused by noise or overshooting).
            lastPeakTime = currentTime;
            return true;
        }
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////
///////////////////////process rrInterval//////////////////////
///////////////////////////////////////////////////////////////////////////
void processrrInterval(float rrInterval) {

    // Add the RR interval to the buffer
    rrIntervals[rrIndex] = rrInterval;
    rrIndex = (rrIndex + 1) % sample_size;  // Update circular buffer index
    if (rrIndex == 19){Warmup = true;}                               // to loose garbage values
}
//////////////////////////////////////////////////////////
//Calculate Beats per minute on 10 samples
/////////////////////////////////////////////////////////
void CalculateBPM(float rrInterval) {
    // Proceed only if the warm-up phase is complete
    if (Warmup) {
        float rrIndex_sum = 0;

        // Sum up the RR intervals
        for (int i = 0; i < sample_size; i++) {
            rrIndex_sum += rrIntervals[i];
        }

        // Calculate Beats Per Minute (BPM)
        if (rrIndex_sum > 0) {
            float beats = 60000.0 / rrIndex_sum;
            bpm = beats * sample_size; // Calculate BPM based on RR intervals
        } else {
            bpm = 0; // Avoid division by zero
        }

    } else {
        // Skip calculation during the warm-up phase
        bpm = 0;
    }
}

void DetectdHeartCondition(float bpm) {
    /////////////////////////////////////////////////////////////////////////////
    // The regularity of the QRS complex intervals can be monitored. 
    // If the intervals between consecutive QRS complexes are too short, it can indicate tachycardia (heart rate over 100 beats a minute). 
    // If the intervals are too long, it can indicate bradycardia (a slow heart rate under 60 bpm).
    /////////////// Tachycardia / Bradycardia Detection /////////////////////////
    /////////////////////////////////////////////////////////////////////////////

    if (bpm > 100) { // Less than 500 ms / more than 100 bpm indicates tachycardia.
        rateAlert = 1;
        // Uncomment to log tachycardia detection:
        // Serial.println("Tachycardia detected");
        // SerialBT.println("Tachycardia detected");
    } else if (bpm < 60) { // More than 1000 ms / less than 60 bpm indicates bradycardia.
        rateAlert = 2;
        // Uncomment to log bradycardia detection:
        // Serial.println("Bradycardia detected");
        // SerialBT.println("Bradycardia detected");
    } else {
        rateAlert = 0; // Heart rate is within the normal range.
        // Uncomment to log normal heart rate:
        // Serial.println("Heart Rate good");
        // SerialBT.println("Heart Rate good");
    }
}

//////////////////////////////////////////////////
///////// Calculate SDNN
/////////////////////////////////////////////////
/*
float calculateSDNN(float rrIntervals[]) {
                        // Calculate SDNN (standard deviation of RR intervals)
                        float successiveDiffSum2 = 0;
                        for (int i = 1; i < sample_size; i++) {
                            float diff = rrIntervals[i] - rrIntervals[i - 1];
                            successiveDiffSum2 += diff;
                        }
                        float Average_successiveDiffSum2 = successiveDiffSum2 / (sample_size-1);

                        float squaredDiffSum = 0;
                        for (int i = 1; i < sample_size; i++) {
                            squaredDiffSum += pow(rrIntervals[i] - rrIntervals[i - 1] - Average_successiveDiffSum2, 2);
                        }
                        return sqrt(squaredDiffSum / (sample_size-1));
}
*/
float calculateRMSSD(float newRR, float previousRR) {
    if (Warmup) {
        // Calculate the difference between successive RR intervals
        float diff = newRR - previousRR;

        // Add the squared difference to the running sum
        successiveDiffSum += pow(diff, 2);

        // Increment the count
        rrCount++;

        // Update Progress
        int percentComplete = (rrCount * 100) / totalRRCount;
        RMSSDProgress = "RMSSD Progress: " + String(percentComplete) + "%";

        // When calculation is complete, save the final value and reset
        if (rrCount >= totalRRCount) {
            finalRMSSD = sqrt(successiveDiffSum / rrCount); // Save final RMSSD
            rrCount = 0;
            successiveDiffSum = 0;
            RMSSDProgress = "RMSSD Calculation Complete"; // Reset progress
        }

        // Return intermediate RMSSD value for real-time updates
        if (rrCount > 1) {
            return sqrt(successiveDiffSum / rrCount);
        } else {
            return 0; // RMSSD is not valid for fewer than 2 intervals
        }
    } else {
        // Skip calculation during the warm-up phase
        return 0;
    }
}


///////////////////////////////////////////////////////////////
////////////FFT calculaions
//////////////////////////////////////////////////////////////
void addRRInterval(float rrInterval) {
    // Skip garbage values during initialization
    if (garbageSkipped < garbageSkipCount) {
        garbageSkipped++;
        Serial.println("Skipping garbage RR interval: " + String(rrInterval));
        return;
    }

    // Add the RR interval to the buffer
    rrBuffer[rrBufferIndex++] = rrInterval;
    fftProgress++; // Increment the progress counter

    // Update StressProgress dynamically
    int percentComplete = (fftProgress * 100) / totalFFTSteps;
    StressProgress = "FFT Progress: " + String(percentComplete) + "%";

    // Reset buffer and progress when FFT is ready
    if (rrBufferIndex >= FFT_SIZE) {
        rrBufferIndex = 0;       // Reset buffer index
        bufferReady = true;      // Mark buffer as ready for processing
        fftProgress = 0;         // Reset progress counter
        StressProgress = "FFT Processing..."; // Indicate FFT processing
    }
}

/////////////////////////////////////////////////////////
//Stress level calculation
//////////////////////////////////////////////////////

void computeFFT() {
    // Prepare FFT inputs
    for (int i = 0; i < FFT_SIZE; i++) {
        vReal[i] = rrBuffer[i];
        vImag[i] = 0;
    }
    // Apply windowing to the data
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);

    // Perform the FFT
    FFT.compute(FFT_FORWARD);

    // Compute magnitudes
    FFT.complexToMagnitude();
}

void analyzeHRV() {
    StressProgress = "Calculating HRV"; // Update progress

    float samplingRate = 4.0;  // Interpolated sampling rate
    float resolution = samplingRate / FFT_SIZE;

    int lfStart = (int)(0.04 / resolution);
    int lfEnd = (int)(0.15 / resolution);
    int hfStart = (int)(0.15 / resolution);
    int hfEnd = (int)(0.4 / resolution);

    float lfPower = calculateBandPower(vReal, lfStart, lfEnd);
    float hfPower = calculateBandPower(vReal, hfStart, hfEnd);
    float lfHfRatio = lfPower / hfPower;

    //Serial.print("LF Power: ");
    //Serial.println(lfPower);
    //Serial.print("HF Power: ");
    //Serial.println(hfPower);
    //Serial.print("LF/HF Ratio: ");
    //Serial.println(lfHfRatio);

    if (lfHfRatio > 2.0) {
        Stress = "High stress level " + String(lfHfRatio);
        StressValue = String(lfHfRatio);
    } else {
        Stress = "Normal stress level " + String(lfHfRatio);
        StressValue = String(lfHfRatio);
    }

    StressProgress = "FFT Ready"; // Indicate HRV completion
    
    // Reset buffer to start over
    bufferReady = false;
    fftProgress = 0; // Reset progress counter
}


float calculateBandPower(float* spectrum, int start, int end) {
    float power = 0;
    for (int i = start; i <= end; i++) {
        power += spectrum[i];
    }
    return power;
}
///////////////////////////////////////////
// logData and store in file
//////////////////////////////////////////
void logData(String timestamp, float heartRate, float rmssd, String stressLevel) {
    // File name to store the logs
    const char* fileName = "/stress_log.csv";

    // Open file for appending
    File file = LittleFS.open(fileName, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }

    // Write the data as a new line
    file.printf("%s,%.2f,%.2f,%s\n", timestamp.c_str(), heartRate, rmssd, stressLevel.c_str());

    file.close();

    // Check storage and manage space
    manageStorage(fileName);
}
//////////////////////////////////////////////////
//Manage storage if Space runs out
///////////////////////////////////////////////
void manageStorage(const char* fileName) {
    const size_t maxFileSize = 1024 * 50; // Set a limit (e.g., 5 KB)

    File file = LittleFS.open(fileName, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }

    size_t fileSize = file.size();
    file.close();

    // If the file size exceeds the limit, delete the oldest entries
    if (fileSize > maxFileSize) {
        File tempFile = LittleFS.open("/temp.csv", FILE_WRITE);
        if (!tempFile) {
            Serial.println("Failed to create temp file");
            return;
        }

        file = LittleFS.open(fileName, FILE_READ);

        size_t bytesToSkip = fileSize / 2; // Remove half of the data
        size_t bytesSkipped = 0;

        while (file.available() && bytesSkipped < bytesToSkip) {
            file.read(); // Skip bytes
            bytesSkipped++;
        }

        // Write the remaining data to the temp file
        while (file.available()) {
            tempFile.write(file.read());
        }

        file.close();
        tempFile.close();

        // Replace the old file with the temp file
        LittleFS.remove(fileName);
        LittleFS.rename("/temp.csv", fileName);

        Serial.println("Old entries deleted to free space");
    }
}
///////////////////////////////////////////////////////////////////
//Safe data to file if High stress is detected
//////////////////////////////////////////////////////////////////
void handleStressDetection(float heartRate, float rmssd, String stressLevel) {
    unsigned long currentMillis = millis();

    // Log data only if 60 seconds have passed since the last log
    if (currentMillis - lastLogTime >= 60000 || lastLogTime == 0) {
        lastLogTime = currentMillis; // Update the last log time

        // Fetch the current RTC time
        DS3231_get(&t);
        // Get the current date and time (you'll need an RTC for this)
        String timestamp = String(t.hour) + ":" + String(t.min) + ":" + String(t.sec) + " " +
                           String(t.mday) + "/" + String(t.mon) + "/" + String(t.year);

        // Log the stress event
        logData(timestamp, heartRate, rmssd, stressLevel);
        Serial.println("Data logged: " + timestamp + ", HeartRate: " + String(heartRate) + 
                       ", RMSSD: " + String(rmssd) + ", StressLevel: " + stressLevel);
    } else {
        Serial.println("High stress detected, but waiting for the next log interval.");
    }
}

