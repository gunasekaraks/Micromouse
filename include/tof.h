#ifndef TOF_H
#define TOF_H

#include <Arduino.h>

class TOF {
public:
    // Constructor
    TOF();

    // Initialize the TOF sensor
    bool begin();

    // Read distance in millimeters
    uint16_t readDistance();

    // Read distance with timeout (returns 0 if timeout)
    uint16_t readDistanceTimeout(uint16_t timeout_ms);

    // Check if sensor is ready
    bool isReady();

    // Set measurement timing budget in microseconds
    void setMeasurementTimingBudget(uint32_t budget_us);

    // Enable/disable continuous mode
    void startContinuous(uint32_t period_ms = 0);
    void stopContinuous();

private:
    bool continuousMode;
    uint32_t timingBudget;
};

#endif // TOF_H
