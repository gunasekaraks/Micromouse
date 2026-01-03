#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "encoder_trigger.h"

class Encoder
{
private:
    // Pin definitions
    int encoderPinA1, encoderPinB1;  // Left motor
    int encoderPinA2, encoderPinB2;  // Right motor

    // Encoder data
    volatile long pulseCount1;  // Left motor
    volatile long pulseCount2;  // Right motor
    float totalDistance1;
    float totalDistance2;

    // State tracking
    volatile bool pulseDetected1;
    volatile bool pulseDetected2;
    volatile int lastDirection1; // 1 = forward, -1 = backward
    volatile int lastDirection2; // 1 = forward, -1 = backward

    // Constants
    float wheelCircumference;
    int pulsesPerRevolution1; // left (357 PPR)
    int pulsesPerRevolution2; // right (207 PPR)

    // Static instance pointer for ISR access
    static Encoder *instance;

    // ISR functions (must be static)
    static void IRAM_ATTR encoderISR1();
    static void IRAM_ATTR encoderISR2();

public:
    // Constructor (supports different PPR for each wheel)
    Encoder(int pinA1 = 35, int pinB1 = 34, int pinA2 = 19, int pinB2 = 26,
        float wheelCirc = 0.034 * 3.14159, int ppr1 = 207, int ppr2 = 357);

    // Initialization
    void begin();

    // Data management
    void reset(bool keepTriggers = false);

    // Getters
    float getDistance1() const;  // Left motor distance
    float getDistance2() const;  // Right motor distance
    float getResultantDistance() const;  // Total distance
    long getPulseCount1() const;
    long getPulseCount2() const;
    int getDirection1() const;
    int getDirection2() const;

    // ISR handlers (called by static ISRs)
    void handleEncoder1();
    void handleEncoder2();

    uint32_t addTrigger(TriggerType type, TriggerCondition condition, float targetValue,
                        std::function<void(float, uint32_t)> callback, bool oneShot);
    void checkTriggers(float leftDistance, float rightDistance);
    bool removeTrigger(uint32_t triggerId);
    void removeAllTriggers();
    bool enableTrigger(uint32_t triggerId);
    bool disableTrigger(uint32_t triggerId);
    uint32_t addDistanceTrigger(float distance, std::function<void()> callback, bool oneShot);
    uint32_t addLeftWheelTrigger(float distance, std::function<void()> callback, bool oneShot);
    uint32_t addRightWheelTrigger(float distance, std::function<void()> callback, bool oneShot);
    uint32_t addRotationTrigger(float wheelBaseDistance, float angleDegrees, std::function<void()> callback, bool oneShot);
};

#endif
