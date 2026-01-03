#include <Arduino.h>
#include "encoder.h"
#include "motor_control.h"

// Create encoder instance with UPDATED motor specs:
// Left motor: pins D35 (A), D34 (B), PPR 207, RPM 630
// Right motor: pins D19 (A), D26 (B), PPR 357, RPM 300
// Wheel diameter: 34mm (0.034m)

Encoder encoder(35, 34, 19, 26, 0.034 * 3.14159, 207, 357);

// Create motor control instance with DRV8833:
// Motor A (Left): IN1=D25, IN2=D13
// Motor B (Right): IN3=D14, IN4=D18
// Sleep pin=D32

MotorControl motorControl(25, 13, 14, 18, 32);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n=== Micromouse Motor & Encoder Control (DRV8833) ===");
    Serial.println("Left Motor (PPR: 207) - Encoder Pins: D35, D34");
    Serial.println("Right Motor (PPR: 357) - Encoder Pins: D19, D26");
    Serial.println("Motor A (Left): IN1=D25, IN2=D13");
    Serial.println("Motor B (Right): IN3=D14, IN4=D18");
    Serial.println("Sleep Pin: D32");
    Serial.println("Wheel Diameter: 34mm");
    Serial.println("===================================================\n");

    // Initialize encoders
    encoder.begin();

    // Initialize motor control with encoder reference
    motorControl.begin(&encoder);

    // Set PID coefficients for straight line movement
    motorControl.setPIDCoefficients(0.8, 0.05, 0.3);

    delay(1000);

    Serial.println("\n=== CONTINUOUS FORWARD MOVEMENT ===");
    Serial.println("Robot will continuously move forward");
    Serial.println("Monitoring encoder readings...\n");
    
    delay(2000);
}

void loop()
{
    // Continuously move forward
    motorControl.moveForward(200);

    // Print real-time distance information
    float leftDistance = encoder.getDistance1() * 100;  // Convert to cm
    float rightDistance = encoder.getDistance2() * 100;  // Convert to cm
    float resultantDistance = encoder.getResultantDistance() * 100;  // Convert to cm

    long leftPulses = encoder.getPulseCount1();
    long rightPulses = encoder.getPulseCount2();

    Serial.print("Left: ");
    Serial.print(leftDistance, 2);
    Serial.print("cm | Right: ");
    Serial.print(rightDistance, 2);
    Serial.print("cm | Resultant: ");
    Serial.print(resultantDistance, 2);
    Serial.print("cm | L:");
    Serial.print(leftPulses);
    Serial.print(" R:");
    Serial.println(rightPulses);
    
    delay(50);  // Reduced delay for smoother operation
}

