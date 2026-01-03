#ifndef ENCODER_TRIGGER_H
#define ENCODER_TRIGGER_H

#include <Arduino.h>
#include <functional>
#include <vector>

// Enum for trigger types
enum TriggerType
{
    TRIGGER_LEFT_WHEEL,
    TRIGGER_RIGHT_WHEEL,
    TRIGGER_BOTH_WHEELS,
    TRIGGER_TOTAL_DISTANCE,
    TRIGGER_DIFFERENTIAL
};

// Enum for trigger conditions
enum TriggerCondition
{
    GREATER_THAN,
    LESS_THAN,
    EQUAL_TO,
    GREATER_EQUAL,
    LESS_EQUAL
};

// Structure for trigger data
struct EncoderTrigger
{
    uint32_t triggerId;
    TriggerType type;
    TriggerCondition condition;
    float targetValue;
    std::function<void(float currentValue, uint32_t triggerId)> callback;
    bool oneShot;
    bool active;
    bool hasTriggered;

    EncoderTrigger() : triggerId(0), type(TRIGGER_LEFT_WHEEL), condition(GREATER_THAN),
                       targetValue(0.0f), oneShot(false), active(true), hasTriggered(false) {}
};

class EncoderTriggerManager
{
private:
    std::vector<EncoderTrigger> triggers;
    uint32_t nextTriggerId;

    bool checkTriggerCondition(const EncoderTrigger &trigger, float currentValue);
    float calculateTriggerValue(TriggerType type, float leftDistance, float rightDistance);

public:
    EncoderTriggerManager();

    // Add trigger handlers
    uint32_t addTrigger(TriggerType type, TriggerCondition condition, float targetValue,
                        std::function<void(float, uint32_t)> callback, bool oneShot = false);

    // Remove triggers
    bool removeTrigger(uint32_t triggerId);
    void removeAllTriggers();

    // Enable/disable triggers
    bool enableTrigger(uint32_t triggerId);
    bool disableTrigger(uint32_t triggerId);

    // Check and execute triggers
    void checkTriggers(float leftDistance, float rightDistance);

    // Reset trigger states
    void resetTriggerStates();

    // Get trigger info
    size_t getTriggerCount() const { return triggers.size(); }
    bool isTriggerActive(uint32_t triggerId);

    // Helper methods for common triggers
    uint32_t addDistanceTrigger(float distance, std::function<void()> callback, bool oneShot = true);
    uint32_t addLeftWheelTrigger(float distance, std::function<void()> callback, bool oneShot = true);
    uint32_t addRightWheelTrigger(float distance, std::function<void()> callback, bool oneShot = true);
    uint32_t addRotationTrigger(float wheelBaseDistance, float angleDegrees, std::function<void()> callback, bool oneShot = true);
};

#endif
