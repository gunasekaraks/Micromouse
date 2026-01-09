#include "turn.h"
#include "motor_control.h"
#include "encoder.h"
#include "wifi_manager.h"
#include "gyro.h"

// External references
extern MotorControl motorControl;
extern Encoder encoder;
extern WiFiManager wifiMgr;

// Robot starting yaw for cardinal heading alignment
// Should be set via setRobotStartingYaw() after gyro stabilizes
static float robotStartingYaw = 0.0f;
static bool startingYawSet = false;

// Set the robot's starting yaw (call after gyro stabilization)
void setRobotStartingYaw(float yaw) {
    robotStartingYaw = yaw;
    startingYawSet = true;
    Serial.print("Robot starting yaw set to: ");
    Serial.println(robotStartingYaw);
}

// Turn parameters
const int TURN_SPEED = 210;              // Maximum speed for turning (lowered to reduce jerk)
const unsigned long TURN_TIMEOUT = 10000; // Maximum time for turn (ms)
const int MIN_TURN_SPEED = 170;          // Raise minimum to avoid stalling mid-turn

// Encoder-based turn parameters
const float WHEEL_DISTANCE = 8.0;        // Distance between wheels in cm
const float WHEEL_DIAMETER = 0.034;      // Wheel diameter in meters (44 mm)
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;  // πd
const float PULSES_PER_ROTATION = 357;   // Encoder pulses per wheel rotation
const float CM_PER_PULSE = (WHEEL_CIRCUMFERENCE * 100) / PULSES_PER_ROTATION;  // cm per pulse
const int DECEL_PULSES = 50;               // Start decelerating within this many pulses remaining

// Encoder-based turn with PID sync (no progressive decel, constant base speed)
const int BASE_TURN_SPEED = 165;          // Constant base speed for both wheels
const float KP_SYNC = 2.0;                // Encoder sync proportional gain
const float KI_SYNC = 0.1;                // Encoder sync integral
const float KD_SYNC = 0.5;                // Encoder sync derivative

// PID variables for encoder sync
float syncIntegral = 0;
float lastSyncError = 0;

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
    
    // Reset encoder and PID controllers
    encoder.reset(true);
    syncIntegral = 0;
    lastSyncError = 0;
    
    unsigned long startTime = millis();
    bool success = false;
    
    while (millis() - startTime < TURN_TIMEOUT) {
        // Update WiFi
        wifiMgr.update();
        
        // Get encoder pulse counts
        long leftPulses = abs(encoder.getPulseCount1());
        long rightPulses = abs(encoder.getPulseCount2());
        
        // Check completion: stop when BOTH wheels reach target (exact same count)
        if (leftPulses >= (long)targetPulses && rightPulses >= (long)targetPulses) {
            motorControl.stop();
            delay(100);
            success = true;
            Serial.print("Turn complete! Pulses: L:");
            Serial.print(leftPulses);
            Serial.print(" R:");
            Serial.print(rightPulses);
            Serial.print(" Target:");
            Serial.print((long)targetPulses);
            Serial.println();
            break;
        }
        
        // Encoder-based PID sync: keep left and right pulses exactly matched
        float syncError = (float)(leftPulses - rightPulses);
        float syncCorrection = KP_SYNC * syncError + 
                              KI_SYNC * syncIntegral + 
                              KD_SYNC * (syncError - lastSyncError);
        syncIntegral += syncError;
        lastSyncError = syncError;
        
        // Calculate remaining pulses for each wheel
        long leftRemaining = (long)targetPulses - leftPulses;
        long rightRemaining = (long)targetPulses - rightPulses;
        
        // Both wheels at base speed with sync correction, slow down wheel that's ahead
        int leftSpeed, rightSpeed;
        if (leftRemaining > 0 && rightRemaining > 0) {
            leftSpeed = constrain(BASE_TURN_SPEED - (int)syncCorrection, MIN_TURN_SPEED, BASE_TURN_SPEED + 20);
            rightSpeed = constrain(BASE_TURN_SPEED + (int)syncCorrection, MIN_TURN_SPEED, BASE_TURN_SPEED + 20);
        } else if (leftRemaining > 0) {
            // Only left wheel needs to continue
            leftSpeed = MIN_TURN_SPEED;
            rightSpeed = 0;
        } else if (rightRemaining > 0) {
            // Only right wheel needs to continue
            leftSpeed = 0;
            rightSpeed = MIN_TURN_SPEED;
        } else {
            leftSpeed = 0;
            rightSpeed = 0;
        }
        
        // Apply turn based on angle direction (positive = right/CW)
        // Motor A = Left, Motor B = Right
        if (angle > 0) {
            // Turn right (CW) - Left forward, Right backward
            motorControl.setMotorASpeed(leftSpeed);
            motorControl.setMotorBSpeed(-rightSpeed);
        } else {
            // Turn left (CCW) - Left backward, Right forward
            motorControl.setMotorASpeed(-leftSpeed);
            motorControl.setMotorBSpeed(rightSpeed);
        }
        
        // Log progress
        String output = "Turn|L:" + String(leftPulses) + "/" + String((long)targetPulses) +
                        "|R:" + String(rightPulses) + "/" + String((long)targetPulses) +
                        "|LSpd:" + String(leftSpeed) + 
                        "|RSpd:" + String(rightSpeed) +
                        "|Sync:" + String(syncCorrection, 1);
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
    
    // Cardinal heading snap: align to nearest 0°, 90°, 180°, or 270° relative to starting yaw
    if (gyro_ok && startingYawSet) {
        updateGyro();
        
        // Calculate 4 cardinal headings relative to starting yaw
        float cardinals[4];
        cardinals[0] = robotStartingYaw;              // 0°
        cardinals[1] = robotStartingYaw + 90.0f;      // +90°
        cardinals[2] = robotStartingYaw + 180.0f;     // +180°
        cardinals[3] = robotStartingYaw - 90.0f;      // -90°
        
        // Normalize all cardinals to [-180, 180]
        for (int i = 0; i < 4; i++) {
            while (cardinals[i] > 180.0f) cardinals[i] -= 360.0f;
            while (cardinals[i] < -180.0f) cardinals[i] += 360.0f;
        }
        
        // Find closest cardinal heading
        float closestCardinal = cardinals[0];
        float minError = 360.0f;
        for (int i = 0; i < 4; i++) {
            float error = cardinals[i] - currentYaw;
            while (error > 180.0f) error -= 360.0f;
            while (error < -180.0f) error += 360.0f;
            if (abs(error) < minError) {
                minError = abs(error);
                closestCardinal = cardinals[i];
            }
        }
        
        Serial.print("Snapping to cardinal: ");
        Serial.print(closestCardinal);
        Serial.print(" (current: ");
        Serial.print(currentYaw);
        Serial.print(", error: ");
        Serial.print(minError);
        Serial.println(")");
        
        // Snap to closest cardinal if error > tolerance
        const float gyroTolerance = 2.5f;
        const int fineSpeed = 160;
        int corrections = 0;
        const int maxCorrections = 50;
        
        while (corrections < maxCorrections) {
            updateGyro();
            float yawError = closestCardinal - currentYaw;
            while (yawError > 180.0f) yawError -= 360.0f;
            while (yawError < -180.0f) yawError += 360.0f;
            
            if (abs(yawError) <= gyroTolerance) {
                Serial.print("Cardinal snap complete. Final yaw: ");
                Serial.println(currentYaw);
                break;
            }
            
            // Apply gentle correction
            if (yawError > 0) {
                motorControl.setMotorASpeed(fineSpeed);
                motorControl.setMotorBSpeed(-fineSpeed);
            } else {
                motorControl.setMotorASpeed(-fineSpeed);
                motorControl.setMotorBSpeed(fineSpeed);
            }
            
            String output = "CardinalSnap|Target:" + String(closestCardinal) + "|Current:" + String(currentYaw) + "|Err:" + String(yawError);
            wifiMgr.sendDataLn(output);
            Serial.println(output);
            
            delay(20);
            corrections++;
        }
        
        motorControl.stop();
        delay(100);
    }
    
    return true;
}
