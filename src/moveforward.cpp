#include <Arduino.h>
#include <Wire.h>
#include "encoder.h"
#include "motor_control.h"
#include "wifi_manager.h"
#include "gyro.h"
#include "moveforward.h"

namespace MoveForward {

// Optional WiFi manager provided by main
static WiFiManager* wifiMgr = nullptr;

// Wheel size: 0.034 m diameter (34 mm)
static constexpr float wheelDiameterMeters = 0.034f;
static constexpr float wheelCircumferenceMeters = wheelDiameterMeters * 3.14159f;

// Encoder instance
static Encoder encoder(35, 34, 19, 26, wheelCircumferenceMeters, 357, 357);

// Motor control instance
static MotorControl motorControl(25, 13, 14, 18, 32);

// Parameters
static int baseSpeed = 160;
static float initialYaw = 0.0f;
static float yawTolerance = 2.0f;   // degrees
static const float moveDistance = 0.18f;  // 18 cm cell length

// Internal helpers
static void disableGyroInterrupt();
static void enableGyroInterrupt();
static void alignToYaw(float targetYaw);
static inline void sendWifiLn(const String& s);

void attachWiFi(WiFiManager* manager)
{
    wifiMgr = manager;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== Micromouse Forward Move (No ToF) ===");
    Serial.println("Waiting for gyro stabilization...");
    Serial.println("====================================\n");

    // Initialize I2C
    Wire.begin();

    // Initialize and stabilize gyro
    setupGyro();
    if (!gyro_ok) {
        Serial.println("ERROR: Gyro initialization failed!");
        while (1) { delay(1000); }
    }

    Serial.println("Gyro initializing, waiting for stable readings...");
    float lastYaw = 0.0f;
    int stableCount = 0;
    while (stableCount < 20) {
        if (updateGyro()) {
            float yawDiff = abs(currentYaw - lastYaw);
            Serial.print("Yaw: ");
            Serial.print(currentYaw);
            Serial.print(" | Diff: ");
            Serial.print(yawDiff);
            Serial.print(" | Stable count: ");
            Serial.println(stableCount);

            if (yawDiff < 0.005f) {
                stableCount++;
            } else {
                stableCount = 0;
            }
            lastYaw = currentYaw;
        }
        delay(20);
    }

    // Store initial yaw
    initialYaw = currentYaw;
    Serial.print("Initial yaw: ");
    Serial.println(initialYaw);

    // Give I2C bus time to settle
    delay(500);

    // Initialize encoders and motors
    encoder.begin();
    motorControl.begin(&encoder);
    motorControl.setPIDCoefficients(2.5f, 0.0005f, 0.00005f);  // PID tuned for pulse counts

    delay(1000);
    Serial.println("Starting forward move...\n");
}

void loop()
{
    // Refresh WiFi connection state if available
    if (wifiMgr) {
        wifiMgr->update();
    }

    // Move forward
    Serial.println("Moving forward 15 cm...");
    encoder.reset(true);

    while (true) {
        // Update WiFi client state
        if (wifiMgr) {
            wifiMgr->update();
        }

        // Update gyro
        updateGyro();

        // Use encoder PID for motor control
        motorControl.moveForward(baseSpeed);
        motorControl.updateStraightLineControl();

        // Read encoder data
        long leftCount = encoder.getPulseCount1();
        long rightCount = encoder.getPulseCount2();
        float leftDist = encoder.getDistance1();
        float rightDist = encoder.getDistance2();
        float avgDist = (abs(leftDist) + abs(rightDist)) / 2.0f;
        long encoderError = leftCount - rightCount + 8;

        // Get motor speeds
        int leftSpeed = motorControl.getCurrentSpeedA();
        int rightSpeed = motorControl.getCurrentSpeedB();

        // Build output string with distance (meters and centimeters) for serial logging
        String output = "L:" + String(leftCount) +
                        "|R:" + String(rightCount) +
                        "|Err:" + String(encoderError) +
                        "|L_Speed:" + String(leftSpeed) +
                        "|R_Speed:" + String(rightSpeed) +
                        "|AvgDist_m:" + String(avgDist, 3) +
                        "|AvgDist_cm:" + String(avgDist * 100.0f, 1);

        // Send to Serial (e.g., Python serial reader)
        Serial.println(output);
        sendWifiLn(output);

        // Check if reached 15cm
        if (avgDist >= moveDistance) {
            motorControl.stop();
            Serial.println("15cm reached; pausing 4s...");
            delay(4000);
            break;
        }

        delay(20);
    }

    delay(200);

    // After move: Check yaw alignment (limited correction)
    updateGyro();
    float yawError = initialYaw - currentYaw;
    while (yawError > 180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    Serial.print("Current yaw: ");
    Serial.print(currentYaw);
    Serial.print(" | Error: ");
    Serial.println(yawError);

    // Only correct if beyond tolerance, but limit total correction angle inside alignToYaw
    if (abs(yawError) > yawTolerance) {
        Serial.println("Yaw drift detected, limited correction...");
        alignToYaw(initialYaw);
    }

    delay(500);  // Pause before next move
}

static void disableGyroInterrupt()
{
    detachInterrupt(digitalPinToInterrupt(33));
}

static void enableGyroInterrupt()
{
    attachInterrupt(digitalPinToInterrupt(33), dmpDataReady, RISING);
}

static void alignToYaw(float targetYaw)
{
    Serial.print("Aligning to yaw: ");
    Serial.println(targetYaw);

    updateGyro();
    float yawError = targetYaw - currentYaw;

    // Normalize error to [-180, 180]
    while (yawError > 180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    if (abs(yawError) < yawTolerance) {
        Serial.println("Already aligned");
        return;
    }

    const int turnSpeed = 110;          // gentler correction
    const float maxCorrection = 8.0f;   // cap correction to avoid big turns
    float corrected = 0.0f;

    while (abs(yawError) > yawTolerance && abs(corrected) < maxCorrection) {
        updateGyro();
        float prevYaw = currentYaw;
        yawError = targetYaw - currentYaw;
        while (yawError > 180.0f) yawError -= 360.0f;
        while (yawError < -180.0f) yawError += 360.0f;

        if (yawError > 0) {
            motorControl.setMotorASpeed(turnSpeed);
            motorControl.setMotorBSpeed(-turnSpeed);
        } else {
            motorControl.setMotorASpeed(-turnSpeed);
            motorControl.setMotorBSpeed(turnSpeed);
        }

        String output = "Yaw_:" + String(currentYaw) + "|Target:" + String(targetYaw) + "|Err:" + String(yawError);
        sendWifiLn(output);
        delay(20);

        // Track how much we've actually rotated to enforce the cap
        updateGyro();
        float delta = currentYaw - prevYaw;
        while (delta > 180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        corrected += delta;
    }

    motorControl.stop();
    delay(200);
    Serial.println("Alignment complete (capped)");
}

static inline void sendWifiLn(const String& s)
{
    if (wifiMgr && wifiMgr->isConnected()) {
        wifiMgr->sendDataLn(s);
    }
}

}  // namespace MoveForward
