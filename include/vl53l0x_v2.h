#ifndef VL53L0X_V2_H
#define VL53L0X_V2_H

#include <stdint.h>
#include <Wire.h>
#include <VL53L0X.h>

// Number of ToF sensors
#define NUM_TOF_SENSORS 3
#define TOF_FRONT 0
#define TOF_RIGHT 1
#define TOF_LEFT 2

extern VL53L0X tofSensor[NUM_TOF_SENSORS];
extern float tof_distance[NUM_TOF_SENSORS];

// Initialize ToF sensors
void setupToF();

// Read all ToF sensors
void readToF();

// Flush ToF readings
void flushToF();

#endif // VL53L0X_V2_H
