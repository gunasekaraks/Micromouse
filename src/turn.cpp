#include "turn.h"
#include "motor_control.h"
#include "encoder.h"
#include "wifi_manager.h"

// External references
extern MotorControl motorControl;
extern Encoder encoder;
extern WiFiManager wifiMgr;

// Turn parameters
const int TURN_SPEED = 220;              // Maximum speed for turning (lowered to reduce jerk)
const unsigned long TURN_TIMEOUT = 8000; // Maximum time for turn (ms)
const int MIN_TURN_SPEED = 170;          // Raise minimum to avoid stalling mid-turn

// Encoder-based turn parameters
const float WHEEL_DISTANCE = 8.0;        // Distance between wheels in cm
const float WHEEL_DIAMETER = 0.034;      // Wheel diameter in meters (34 mm)
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;  // πd
const float PULSES_PER_ROTATION = 357;   // Encoder pulses per wheel rotation
const float CM_PER_PULSE = (WHEEL_CIRCUMFERENCE * 100) / PULSES_PER_ROTATION;  // cm per pulse
const int DECEL_PULSES = 50;               // Start decelerating within this many pulses remaining

// PID for encoder-based turn (encoder only, no gyro in control loop)
const float KP_ENCODER = 2.0;
const float KI_ENCODER = 0.01;
const float KD_ENCODER = 0.4;

// PID variables
float encoderIntegral = 0;
float lastEncoderError = 0;

// Gyro-free wrapper: reuse encoder-based turn for arbitrary angle
bool turn(float angle) {
    return turnEncoderBased(angle);
}

bool turnRight90() {
    Serial.println("=== Turning Right 90° (encoder only) ===");
    return turnEncoderBased(TURN_RIGHT);
}

bool turnLeft90() {
    Serial.println("=== Turning Left 90° ===");
    return turnEncoderBased(TURN_LEFT);
}

bool turn180() {
    Serial.println("=== Turning 180° ===");
    return turnEncoderBased(TURN_180);
}

// Encoder-based turn with PID and gyro feedback
bool turnEncoderBased(float angle) {
    // Calculate target pulse count
    // For a turn: each wheel travels arc_length = (wheel_distance/2) * angle_radians
    float angleRad = abs(angle) * 3.14159 / 180.0;
    float arcLength = (WHEEL_DISTANCE / 2.0) * angleRad;  // cm
    float targetPulses = arcLength / CM_PER_PULSE;
    
    Serial.print("Turning ");
    Serial.print(angle);
    Serial.print("° (");
    Serial.print(targetPulses);
    Serial.println(" pulses)");
    
    // Reset encoder and PID
    encoder.reset(true);
    encoderIntegral = 0;
    lastEncoderError = 0;
    
    unsigned long startTime = millis();
    float commandedSpeed = 0;  // Smoothed speed command to avoid sudden jerks
    bool success = false;
    
    while (millis() - startTime < TURN_TIMEOUT) {
        // Update WiFi
        wifiMgr.update();
        
        // Get encoder data (use absolute values for comparison)
        long leftPulses = abs(encoder.getPulseCount1());
        long rightPulses = abs(encoder.getPulseCount2());
        long metricPulses = max(leftPulses, rightPulses);  // control by leading wheel to avoid overshoot
        
        // Calculate encoder error
        float encoderError = targetPulses - metricPulses;
        
        // Check if we've reached the target (encoder-based only)
        // Stop early (20 pulses before) to account for momentum
        if (encoderError <= 20) {
            motorControl.stop();
            delay(100);  // Brief hold to prevent creep from momentum
            success = true;
            Serial.print("Turn complete! Pulses: ");
            Serial.print(metricPulses + 20);  // Report what we likely reached by now
            Serial.print("/");
            Serial.print((long)targetPulses);
            Serial.println();
            break;
        }
        
        // Compute a speed target from remaining pulses (P-only magnitude)
        // Keep direction fixed by desired turn side
        float targetSpeed = KP_ENCODER * encoderError; // encoderError >= 0

        // Ensure minimum speed - keep motors moving
        if (targetSpeed < MIN_TURN_SPEED) targetSpeed = MIN_TURN_SPEED;

        // Limit maximum speed
        if (targetSpeed > TURN_SPEED) targetSpeed = TURN_SPEED;

        // Decelerate as we approach the target to avoid overshoot
        if (encoderError < DECEL_PULSES) {
            float decelFactor = encoderError / (float)DECEL_PULSES; // 1 -> 0
            float maxAllowed = MIN_TURN_SPEED + decelFactor * (TURN_SPEED - MIN_TURN_SPEED);
            if (targetSpeed > maxAllowed) targetSpeed = maxAllowed;
        }

        // Apply a small acceleration limit to soften starts/changes
        float maxDelta = 16;  // allow slightly faster ramp to avoid stall
        float delta = targetSpeed - commandedSpeed;
        if (delta > maxDelta) delta = maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;
        commandedSpeed += delta;
        
        // Apply turn based on angle direction (positive = right/CW)
        // Motor A = Left, Motor B = Right
        if (angle > 0) {
            // Turn right (CW) - Left forward, Right backward
            motorControl.setMotorASpeed((int)commandedSpeed);
            motorControl.setMotorBSpeed(-(int)commandedSpeed);
        } else {
            // Turn left (CCW) - Left backward, Right forward
            motorControl.setMotorASpeed(-(int)commandedSpeed);
            motorControl.setMotorBSpeed((int)commandedSpeed);
        }
        
        // Log progress with more detail
        String output = "EncTurn|Pulses:" + String(metricPulses) + "/" + String((long)targetPulses) +
                        "|EncErr:" + String(encoderError, 1) +
                        "|L:" + String(leftPulses) + 
                        "|R:" + String(rightPulses) +
                        "|Speed:" + String((int)commandedSpeed);
        wifiMgr.sendDataLn(output);
        Serial.println(output);
        
        delay(20);
    }
    
    // Ensure motors are stopped
    motorControl.stop();
    delay(200);
    
    if (!success) {
        Serial.println("Turn timeout!");
        return false;
    }
    
    return true;
}
