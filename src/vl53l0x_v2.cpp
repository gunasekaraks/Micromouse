#include "vl53l0x_v2.h"

// Three ToF sensors using VL53L0X library
VL53L0X tofSensor[NUM_TOF_SENSORS];
float tof_distance[NUM_TOF_SENSORS] = {0, 0, 0};
float tof_filtered[NUM_TOF_SENSORS] = {0, 0, 0};  // Filtered values
bool tof_initialized[NUM_TOF_SENSORS] = {false, false, false};  // Track if filter is initialized

// XSHUT pins for each sensor
const int XSHUT_PINS[NUM_TOF_SENSORS] = {17, 16, 5};  // Front, Right, Left
const uint8_t I2C_ADDRESSES[NUM_TOF_SENSORS] = {0x30, 0x31, 0x32};  // Custom addresses (not 0x29)
const float MAX_RANGE = 1200.0f;
const float CALIBRATION_OFFSET[NUM_TOF_SENSORS] = {0.0f, 0.0f, 0.0f};  // Adjust per sensor
const float FILTER_ALPHA = 0.3f;  // Filter coefficient (0.1-0.5, lower = smoother)


void setupToF() {
    Serial.println("Initializing ToF sensors...");
    
    // Pull all XSHUT pins low first
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        pinMode(XSHUT_PINS[i], OUTPUT);
        digitalWrite(XSHUT_PINS[i], LOW);
    }
    delay(20);
    
    // Initialize each sensor one by one
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        // Enable this sensor
        digitalWrite(XSHUT_PINS[i], HIGH);
        delay(20);
        
        // Initialize sensor
        if (!tofSensor[i].init()) {
            Serial.print("ToF Error ");
            Serial.println(i + 1);
        } else {
            Serial.print("ToF Sensor initialized ");
            Serial.println(i + 1);
        }
        
        // Set custom address immediately
        tofSensor[i].setAddress(I2C_ADDRESSES[i]);
        tofSensor[i].setMeasurementTimingBudget(20000);
        tofSensor[i].startContinuous();
    }
    
    Serial.println("All ToF sensors ready");
}


void readToF() {
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        float distance = tofSensor[i].readRangeContinuousMillimeters();
        
        // Check for timeout or out of range
        if (tofSensor[i].timeoutOccurred() || distance == 0) {
            // Skip this reading, keep previous filtered value
            continue;
        }
        else if (distance >= 1200.0f) {
            distance = MAX_RANGE;
        }
        else {
            distance = distance - CALIBRATION_OFFSET[i];
            if (distance < 0) distance = 0;
        }
        
        // Apply exponential moving average filter
        if (!tof_initialized[i]) {
            // First valid reading, initialize filter
            tof_filtered[i] = distance;
            tof_initialized[i] = true;
        } else {
            // EMA: filtered = alpha * new + (1 - alpha) * previous
            tof_filtered[i] = FILTER_ALPHA * distance + (1.0f - FILTER_ALPHA) * tof_filtered[i];
        }
        
        // Update output with filtered value
        tof_distance[i] = tof_filtered[i];
    }
}


void flushToF() {
    // Clear the sensor buffer by reading multiple times
    for (int i = 0; i < 10; i++) {
        readToF();
        delay(5);
    }
}
