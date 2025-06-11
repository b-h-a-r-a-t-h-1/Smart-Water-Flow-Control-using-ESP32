#include <WiFi.h>
#include <time.h>

// Wi-Fi credentials
const char* ssid = "Batman";             // Wi-Fi SSID
const char* password = "12345678";       // Wi-Fi password

// Define A4988 driver pins
#define STEP_PIN    18
#define DIR_PIN     19
#define ENABLE_PIN  23

// Define PIR sensor pin
#define PIR_PIN     4   // Connect PIR sensor output to GPIO4

// Define built-in LED pin (GPIO2 for ESP32)
#define LED_PIN     2

// Motor parameters
const int stepsPerRevolution = 200;
const int microstepping = 1;
const int motorSpeed = 700;

// PIR sensor timing
const unsigned long motionTimeout = 10000;  // 10 seconds timeout if no motion is detected
unsigned long lastMotionTime = 0;           // Stores the last time motion was detected
bool motorInOriginalPosition = true;        // Tracks if the motor is in its original position
bool motionHandled = false;                 // Tracks if the motion event has been handled

// Define minute intervals for motor activation
struct MinuteInterval {
    int startMinute;   // Start minute of the interval
    int endMinute;     // End minute of the interval
};

// Motor conditions a,b,c,d,e,f are the time intervals which we need to calibrate for our requirements
// here I create 3 intervals for my purpose
MinuteInterval condition1 = {a, b};
MinuteInterval condition2 = {c, d};
MinuteInterval condition3 = {e, f};

void setup() {
    // Set up pins
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(PIR_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT); // Built-in LED

    // Turn off LED initially
    digitalWrite(LED_PIN, LOW);

    // Enable motor driver
    digitalWrite(ENABLE_PIN, LOW);

    // Initialize Serial
    Serial.begin(115200);

    // Connect to Wi-Fi
    connectToWiFi();

    // Set RTC to IST (UTC+5:30)
    configTime(19800, 0, "pool.ntp.org", "time.nist.gov"); // 19800 seconds = 5 hours 30 minutes
    Serial.println("Waiting for time synchronization...");
    delay(2000);
    printLocalTime();
}

void loop() {
    // Get current time
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return;
    }

    int currentMinute = timeinfo.tm_min;

    // Check if the time is within any active interval
    bool isActive = isWithinMinuteInterval(currentMinute, condition1) ||
                    isWithinMinuteInterval(currentMinute, condition2) ||
                    isWithinMinuteInterval(currentMinute, condition3);

    int rotationSteps = 0;

    if (isWithinMinuteInterval(currentMinute, condition1)) {
        rotationSteps = stepsPerRevolution / 4; // 90° rotation
    } else if (isWithinMinuteInterval(currentMinute, condition2)) {
        rotationSteps = stepsPerRevolution / 2; // 180° rotation
    } else if (isWithinMinuteInterval(currentMinute, condition3)) {
        rotationSteps = stepsPerRevolution;     // 360° rotation
    }

    if (isActive) {
        // Check PIR sensor for motion
        bool motionDetected = digitalRead(PIR_PIN);
        if (motionDetected) {
            lastMotionTime = millis(); // Reset timeout timer

            // Turn on the built-in LED
            digitalWrite(LED_PIN, HIGH);

            if (motorInOriginalPosition && !motionHandled) {
                // Handle motion only once per interval
                Serial.println("Motion detected! Rotating motor.");
                rotateMotor(rotationSteps, HIGH); // Rotate as per the active interval
                motionHandled = true;             // Mark the motion as handled
                motorInOriginalPosition = false;
            }
        } else if (!motorInOriginalPosition && millis() - lastMotionTime >= motionTimeout) {
            Serial.println("No motion detected for 10 seconds. Returning to original position.");
            rotateMotor(rotationSteps, LOW);     // Rotate back to original position
            motorInOriginalPosition = true;      // Motor returned to original position
            motionHandled = false;               // Reset motion flag for the next interval

            // Turn off the built-in LED
            digitalWrite(LED_PIN, LOW);
        }
    } else if (!motorInOriginalPosition) {
        // If outside active interval, ensure motor is in the original position
        Serial.println("Outside active interval. Returning motor to original position.");
        rotateMotor(rotationSteps, LOW);
        motorInOriginalPosition = true;
        motionHandled = false;

        // Turn off the built-in LED
        digitalWrite(LED_PIN, LOW);
    }

    delay(1000);
}

// Function to check if current minute is within a given interval
bool isWithinMinuteInterval(int currentMinute, MinuteInterval interval) {
    return (currentMinute >= interval.startMinute && currentMinute < interval.endMinute);
}

// Function to rotate the motor
void rotateMotor(int steps, bool direction) {
    if (direction == HIGH) {
        Serial.println("Motor rotating clockwise.");
    } else {
        Serial.println("Motor rotating counterclockwise.");
    }

    digitalWrite(DIR_PIN, direction); // Set motor direction

    for (int i = 0; i < steps * microstepping; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(1000000 / motorSpeed / 2);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(1000000 / motorSpeed / 2);
    }
}

// Connect to Wi-Fi
void connectToWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi connected");
}

// Print the current local time
void printLocalTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return;
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}
