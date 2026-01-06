#include "vl53l0x_v2.h"

// VL53L0XV2 Register definitions
#define VL53L0X_REG_SYSRANGE_START          0x00
#define VL53L0X_REG_SYSTEM_THRESH_HIGH      0x0C
#define VL53L0X_REG_SYSTEM_THRESH_LOW       0x0E
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG  0x01
#define VL53L0X_REG_SYSRANGE_MODE_START     0x00
#define VL53L0X_REG_RESULT_RANGE_STATUS     0x13
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN  0x06
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN   0x08
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF  0x0A
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF   0x0C
#define VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF            0x0E
#define VL53L0X_REG_RESULT_RANGE_VAL                       0x14
#define VL53L0X_REG_RESULT_RANGE_RAW                       0x1A

#define VL53L0X_ADDRESS_DEFAULT             0x29
#define VL53L0X_WHO_AM_I                    0xC0
#define VL53L0X_WHO_AM_I_VALUE              0xEE

VL53L0XV2::VL53L0XV2(uint8_t xshutPin)
    : _address(VL53L0X_ADDRESS_DEFAULT), _xshutPin(xshutPin), _initialized(false) {
}

bool VL53L0XV2::begin(uint8_t address) {
    _address = address;
    
    // Initialize XSHUT pin
    pinMode(_xshutPin, OUTPUT);
    digitalWrite(_xshutPin, LOW);
    delay(100);
    digitalWrite(_xshutPin, HIGH);
    delay(100);
    
    // Note: Wire.begin() should be called in main setup before this
    
    // Check WHO_AM_I register
    uint8_t whoami = readReg(VL53L0X_WHO_AM_I);
    if (whoami != VL53L0X_WHO_AM_I_VALUE) {
        return false;
    }
    
    // Initialize sensor with default settings
    writeReg(VL53L0X_REG_SYSRANGE_START, 0x00);
    
    _initialized = true;
    return true;
}

uint16_t VL53L0XV2::getDistance() {
    if (!_initialized) {
        return 0;
    }
    
    // Start single ranging measurement
    writeReg(0x00, 0x01);
    
    // Wait for measurement to complete (check bit 0 of register 0x13)
    uint8_t status;
    int timeout = 0;
    do {
        status = readReg(0x13);
        delay(1);
        timeout++;
        if (timeout > 100) {
            return 8191; // Return max range on timeout
        }
    } while ((status & 0x01) == 0);
    
    // Read distance value from register 0x1E (RESULT_RANGE_VAL in mm)
    uint16_t distance = readReg16(0x1E);
    
    // Clear interrupt
    writeReg(0x0B, 0x01);
    
    return distance;
}

bool VL53L0XV2::readRaw(uint8_t *data, size_t length) {
    if (!_initialized || !data || length == 0) {
        return false;
    }
    
    Wire.beginTransmission(_address);
    Wire.write(VL53L0X_REG_RESULT_RANGE_STATUS);
    Wire.endTransmission();
    
    Wire.requestFrom(_address, (uint8_t)length);
    
    for (size_t i = 0; i < length; i++) {
        if (Wire.available()) {
            data[i] = Wire.read();
        } else {
            return false;
        }
    }
    
    return true;
}

void VL53L0XV2::setAddress(uint8_t address) {
    _address = address;
}

uint8_t VL53L0XV2::getAddress() {
    return _address;
}

bool VL53L0XV2::startRanging() {
    if (!_initialized) {
        return false;
    }
    
    writeReg(VL53L0X_REG_SYSRANGE_START, 0x01);
    return true;
}

void VL53L0XV2::stopRanging() {
    writeReg(VL53L0X_REG_SYSRANGE_START, 0x00);
}

bool VL53L0XV2::isDataReady() {
    uint8_t status = readReg(VL53L0X_REG_RESULT_RANGE_STATUS);
    return (status & 0x01) == 0;
}

uint8_t VL53L0XV2::readReg(uint8_t reg) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(_address, (uint8_t)1);
    
    if (Wire.available()) {
        return Wire.read();
    }
    
    return 0;
}

void VL53L0XV2::writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint16_t VL53L0XV2::readReg16(uint8_t reg) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(_address, (uint8_t)2);
    
    uint16_t value = 0;
    if (Wire.available()) {
        value = (Wire.read() << 8);
        if (Wire.available()) {
            value |= Wire.read();
        }
    }
    
    return value;
}

void VL53L0XV2::writeReg16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}
