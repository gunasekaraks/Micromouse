#include <Arduino.h>
#include <Wire.h>
#include "encoder.h"
#include "motor_control.h"
#include "wifi_manager.h"
#include "gyro.h"
#include "vl53l0x_v2.h"
#include "turn.h"

// WiFi Manager
WiFiManager wifiMgr("slt ftth", "71091044");

// Encoder instance
Encoder encoder(35, 34, 19, 26, 0.034 * 3.14159, 357, 357);

// Motor control instance
MotorControl motorControl(25, 13, 14, 18, 32);

// ToF sensor is declared in vl53l0x_v2.cpp

// Parameters
int baseSpeed = 160;
float initialYaw = 0.0;
float yawTolerance = 1;  // degrees
const float moveDistance = 0.20;  // 20cm in meters
unsigned long lastTofReadTime = 0;  // Track when we last read ToF

// Forward declarations
void disableGyroInterrupt();
void enableGyroInterrupt();
void alignToYaw(float targetYaw);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== Micromouse Navigation with Gyro & ToF ===");
    Serial.println("Waiting for gyro stabilization...");
    Serial.println("============================================\n");

    // Initialize WiFi
    if (!wifiMgr.begin()) {
        Serial.println("Warning: WiFi not connected, continuing with Serial only\n");
    }

    // Initialize I2C
    Wire.begin();
    
    // Initialize and stabilize gyro
    setupGyro();
    if (!gyro_ok) {
        Serial.println("ERROR: Gyro initialization failed!");
        while(1) { delay(1000); }  // Halt
    }
    
    Serial.println("Gyro initializing, waiting for stable readings...");
    float lastYaw = 0;
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
            
            if (yawDiff < 0.005) {
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
    
    // Initialize ToF sensor (disable gyro interrupt first)
    disableGyroInterrupt();
    delay(100);
    
    setupToF();
    enableGyroInterrupt();
    
    // Flush ToF readings for 1 second
    Serial.println("Flushing ToF sensor...");
    disableGyroInterrupt();
    flushToF();
    enableGyroInterrupt();
    
    Serial.println("ToF sensor ready\n");

    // Initialize encoders and motors
    encoder.begin();
    motorControl.begin(&encoder);
    motorControl.setPIDCoefficients(2.5, 0.0005, 0.00005);  // PID tuned for pulse counts
    
    delay(1000);
    Serial.println("Starting navigation...\n");
}

void disableGyroInterrupt() {
    detachInterrupt(digitalPinToInterrupt(33));  // Detach from interrupt pin
}

void enableGyroInterrupt() {
    attachInterrupt(digitalPinToInterrupt(33), dmpDataReady, RISING);  // Re-attach interrupt
}

void alignToYaw(float targetYaw) {
    Serial.print("Aligning to yaw: ");
    Serial.println(targetYaw);
    
    updateGyro();
    float yawError = targetYaw - currentYaw;
    
    // Normalize error to [-180, 180]
    while (yawError > 180) yawError -= 360;
    while (yawError < -180) yawError += 360;
    
    if (abs(yawError) < yawTolerance) {
        Serial.println("Already aligned");
        return;
    }
    
    // Turn until aligned
    int turnSpeed = 160;
    while (abs(yawError) > yawTolerance) {
        updateGyro();
        yawError = targetYaw - currentYaw;
        while (yawError > 180) yawError -= 360;
        while (yawError < -180) yawError += 360;
        
        if (yawError > 0) {
            // Turn left (CCW)
            motorControl.setMotorASpeed(turnSpeed);
            motorControl.setMotorBSpeed(-turnSpeed);
        } else {
            // Turn right (CW)
            motorControl.setMotorASpeed(-turnSpeed);
            motorControl.setMotorBSpeed(turnSpeed);
        }
        
        String output = "Yaw_:" + String(currentYaw) + "|Target:" + String(targetYaw) + "|Err:" + String(yawError);
        wifiMgr.sendDataLn(output);
        delay(20);
    }
    
    motorControl.stop();
    delay(200);
    Serial.println("Alignment complete");
}

void loop()
{
    // Update WiFi connections
    wifiMgr.update();
    
    // Test 90° Right Turn (Encoder-based with PID and Gyro feedback)
    Serial.println("\n\n========== TEST: RIGHT 90° TURN (ENCODER-BASED) ==========");
    turnEncoderBased(90);  // Positive = right turn
    delay(2000);
}




