#include <Arduino.h>
#include <Wire.h>
#include "vl53l0x_v2.h"

VL53L0XV2 tofSensor(17);  // XSHUT pin 17

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== ToF Sensor Test ===\n");
    
    // Initialize I2C
    Wire.begin();
    delay(500);
    
    // Setup XSHUT pin
    pinMode(17, OUTPUT);
    digitalWrite(17, HIGH);  // Enable ToF
    delay(200);
    
    // Initialize ToF sensor
    if (!tofSensor.begin()) {
        Serial.println("ERROR: ToF sensor initialization failed!");
        while(1) { delay(1000); }
    }
    
    Serial.println("ToF sensor ready!");
    Serial.println("Reading distance every 200ms...\n");
}

void loop() {
    uint16_t distance = tofSensor.getDistance();
    
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
    
    delay(200);
}
