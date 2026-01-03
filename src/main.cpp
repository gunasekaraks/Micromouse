#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== DRV8833 Motor Driver Diagnostic Test ===\n");

    // Pin definitions
    int AIN1 = 25;
    int AIN2 = 13;
    int BIN1 = 14;
    int BIN2 = 18;
    int STBY = 32;git branch -M main



    // Set as outputs
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);

    // Enable DRV8833 (Sleep pin HIGH)
    digitalWrite(STBY, HIGH);
    Serial.println("1. Sleep pin (D32) set to HIGH");
    delay(500);

    // Setup PWM
    const int PWM_FREQ = 20000;
    const int PWM_RESOLUTION = 8;
    
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(AIN1, 0);
    ledcAttachPin(AIN2, 1);
    ledcAttachPin(BIN1, 2);
    ledcAttachPin(BIN2, 3);

    Serial.println("2. PWM channels configured (20kHz)\n");

    // Test Motor A (Left)
    Serial.println("Testing Motor A (Left):");
    Serial.println("- Setting AIN1 = 255, AIN2 = 0 (Forward at MAX speed)");
    ledcWrite(0, 255);  // AIN1 PWM - FULL SPEED
    ledcWrite(1, 0);    // AIN2 = 0
    delay(2000);
    Serial.println("- Motor should be spinning forward at full speed\n");

    ledcWrite(0, 0);
    ledcWrite(1, 0);
    delay(1000);

    // Test Motor B (Right)
    Serial.println("Testing Motor B (Right):");
    Serial.println("- Setting BIN1 = 255, BIN2 = 0 (Forward at MAX speed)");
    ledcWrite(2, 255);  // BIN1 PWM - FULL SPEED
    ledcWrite(3, 0);    // BIN2 = 0
    delay(2000);
    Serial.println("- Motor should be spinning forward at full speed\n");

    ledcWrite(2, 0);
    ledcWrite(3, 0);

    Serial.println("=== Test Complete ===");
    Serial.println("If motors didn't spin:");
    Serial.println("1. Check OUT pins are connected to motors");
    Serial.println("2. Check motor power supply voltage");
    Serial.println("3. Check GND connection between ESP32 and DRV8833");
}

void loop()
{
    delay(1000);
}

