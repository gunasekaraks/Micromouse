#ifndef VL53L0X_V2_H
#define VL53L0X_V2_H

#include <stdint.h>
#include <Wire.h>

class VL53L0XV2 {
public:
    // Constructor
    VL53L0XV2(uint8_t xshutPin = 17);
    
    // Initialize sensor
    bool begin(uint8_t address = 0x29);
    
    // Read distance in millimeters
    uint16_t getDistance();
    
    // Read raw data
    bool readRaw(uint8_t *data, size_t length);
    
    // Set I2C address
    void setAddress(uint8_t address);
    
    // Get current I2C address
    uint8_t getAddress();
    
    // Start continuous ranging
    bool startRanging();
    
    // Stop continuous ranging
    void stopRanging();
    
    // Check if data is ready
    bool isDataReady();

private:
    uint8_t _address;
    uint8_t _xshutPin;
    bool _initialized;
    
    // Register read/write helpers
    uint8_t readReg(uint8_t reg);
    void writeReg(uint8_t reg, uint8_t value);
    uint16_t readReg16(uint8_t reg);
    void writeReg16(uint8_t reg, uint16_t value);
};

#endif // VL53L0X_V2_H
