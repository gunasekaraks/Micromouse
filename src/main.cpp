#include <Arduino.h>
#include <Wire.h>
#include "encoder.h"
#include "motor_control.h"
#include "wifi_manager.h"
#include "gyro.h"
#include "vl53l0x_v2.h"

// WiFi Manager
WiFiManager wifiMgr("Amiru", "amiru123");

// Encoder instance
Encoder encoder(35, 34, 19, 26, 0.034 * 3.14159, 357, 357);

// Motor control instance
MotorControl motorControl(25, 13, 14, 18, 32);

// ToF sensor
VL53L0XV2 tofSensor(17);  // XSHUT pin 17

// Parameters
int baseSpeed = 180;
float initialYaw = 0.0;
float yawTolerance = 2.0;  // degrees
const float moveDistance = 0.20;  // 20cm in meters

// Forward declarations
void disableGyroInterrupt();
void enableGyroInterrupt();
void disableToF();
void enableToF();
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
    
    // Enable ToF (pull XSHUT high)
    pinMode(17, OUTPUT);
    enableToF();
    delay(200);
    
    if (!tofSensor.begin()) {
        Serial.println("ERROR: ToF sensor initialization failed!");
        enableGyroInterrupt();
        while(1) { delay(1000); }  // Halt
    }
    Serial.println("ToF sensor ready");
    enableGyroInterrupt();
    
    // Print ToF readings for 3 seconds (pause gyro to avoid I2C conflicts)
    Serial.println("Reading ToF for 3 seconds...");
    disableGyroInterrupt();
    delay(100);
    enableToF();  // Ensure ToF is powered
    delay(200);
    unsigned long tofStartTime = millis();
    while (millis() - tofStartTime < 3000) {
        uint16_t distance = tofSensor.getDistance();
        Serial.print("ToF Distance: ");
        Serial.print(distance);
        Serial.println(" mm");
        delay(200);
    }
    Serial.println("ToF test complete\n");
    enableGyroInterrupt();
    
    // Give I2C time to settle again
    delay(500);

    // Initialize encoders and motors
    encoder.begin();
    motorControl.begin(&encoder);
    motorControl.setPIDCoefficients(2.5, 0.0, 0.0);  // PID tuned for pulse counts
    
    delay(1000);
    Serial.println("Starting navigation...\n");
}

void disableToF() {
    digitalWrite(17, LOW);  // Pull XSHUT low to power down ToF
}

void enableToF() {
    digitalWrite(17, HIGH);  // Pull XSHUT high to enable ToF
    delay(100);  // Wait for ToF to boot
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
    
    // Move 20cm forward
    Serial.println("Moving 20cm forward...");
    encoder.reset(true);
    
    while (true) {
        // Update WiFi
        wifiMgr.update();
        
        // Use encoder PID for motor control
        motorControl.moveForward(baseSpeed);
        motorControl.updateStraightLineControl();
        
        // Read encoder data
        long leftCount = encoder.getPulseCount1();
        long rightCount = encoder.getPulseCount2();
        float leftDist = encoder.getDistance1();
        float rightDist = encoder.getDistance2();
        float avgDist = (abs(leftDist) + abs(rightDist)) / 2.0;
        
        // Get motor speeds
        int leftSpeed = motorControl.getCurrentSpeedA();
        int rightSpeed = motorControl.getCurrentSpeedB();
        
        // Build output string (no gyro during movement)
        String output = "L:" + String(leftCount) + 
                        "|R:" + String(rightCount) + 
                        "|Dist:" + String(avgDist, 3) +
                        "|L_Speed:" + String(leftSpeed) +
                        "|R_Speed:" + String(rightSpeed);
        
        // Send to Serial and WiFi
        wifiMgr.sendDataLn(output);
        
        // Check if reached 20cm
        if (avgDist >= moveDistance) {
            motorControl.stop();
            Serial.println("20cm reached");
            break;
        }
        
        delay(20);
    }
    
    delay(200);
    
    // After 20cm: Check yaw alignment
    updateGyro();
    float yawError = initialYaw - currentYaw;
    while (yawError > 180) yawError -= 360;
    while (yawError < -180) yawError += 360;
    
    Serial.print("Current yaw: ");
    Serial.print(currentYaw);
    Serial.print(" | Error: ");
    Serial.println(yawError);
    
    if (abs(yawError) > yawTolerance) {
        Serial.println("Yaw drift detected, correcting...");
        alignToYaw(initialYaw);
    }
    
    delay(200);
    
    // Check for obstacle (pause gyro to avoid I2C conflicts)
    disableGyroInterrupt();
    delay(50);
    enableToF();  // Ensure ToF is powered before reading
    delay(100);
    uint16_t distance = tofSensor.getDistance();
    enableGyroInterrupt();
    
    Serial.print("ToF distance: ");
    Serial.print(distance);
    Serial.println(" mm");
    
    if (distance < 60) {
        Serial.println("Obstacle detected! Stopping.");
        motorControl.stop();
        String output = "OBSTACLE|Dist:" + String(distance) + "mm";
        wifiMgr.sendDataLn(output);
        delay(1000);
        return;  // Stay stopped
    }
    
    delay(500);  // Pause before next move
}




