#include "encoder_trigger.h"

EncoderTriggerManager::EncoderTriggerManager() : nextTriggerId(1)
{
    triggers.reserve(10); // Pre-allocate to prevent memory fragmentation
}

bool EncoderTriggerManager::checkTriggerCondition(const EncoderTrigger &trigger, float currentValue)
{
    switch (trigger.condition)
    {
    case GREATER_THAN:
        return currentValue > trigger.targetValue;
    case LESS_THAN:
        return currentValue < trigger.targetValue;
    case EQUAL_TO:
        return abs(currentValue - trigger.targetValue) < 0.001f;
    case GREATER_EQUAL:
        return currentValue >= trigger.targetValue;
    case LESS_EQUAL:
        return currentValue <= trigger.targetValue;
    default:
        return false;
    }
}

float EncoderTriggerManager::calculateTriggerValue(TriggerType type, float leftDistance, float rightDistance)
{
    switch (type)
    {
    case TRIGGER_LEFT_WHEEL:
        return leftDistance;
    case TRIGGER_RIGHT_WHEEL:
        return rightDistance;
    case TRIGGER_BOTH_WHEELS:
        return (leftDistance + rightDistance) / 2.0f;
    case TRIGGER_TOTAL_DISTANCE:
        return abs(leftDistance) + abs(rightDistance);
    case TRIGGER_DIFFERENTIAL:
        return leftDistance - rightDistance;
    default:
        return 0.0f;
    }
}

uint32_t EncoderTriggerManager::addTrigger(TriggerType type, TriggerCondition condition,
                                           float targetValue, std::function<void(float, uint32_t)> callback,
                                           bool oneShot)
{
    if (!callback)
    {
        Serial.println("Error: Invalid callback provided to addTrigger");
        return 0;
    }

    if (triggers.size() >= 20)
    {
        Serial.println("Error: Maximum number of triggers reached");
        return 0;
    }

    EncoderTrigger trigger;
    trigger.triggerId = nextTriggerId++;
    trigger.type = type;
    trigger.condition = condition;
    trigger.targetValue = targetValue;
    trigger.callback = callback;
    trigger.oneShot = oneShot;
    trigger.active = true;
    trigger.hasTriggered = false;

    triggers.push_back(trigger);

    Serial.print("Added encoder trigger ID: ");
    Serial.print(trigger.triggerId);
    Serial.print(", Type: ");
    Serial.print((int)type);
    Serial.print(", Target: ");
    Serial.println(targetValue, 2);

    return trigger.triggerId;
}

bool EncoderTriggerManager::removeTrigger(uint32_t triggerId)
{
    for (auto it = triggers.begin(); it != triggers.end(); ++it)
    {
        if (it->triggerId == triggerId)
        {
            Serial.print("Removing trigger ID: ");
            Serial.println(triggerId);
            triggers.erase(it);
            return true;
        }
    }
    Serial.print("Warning: Trigger ID not found for removal: ");
    Serial.println(triggerId);
    return false;
}

void EncoderTriggerManager::removeAllTriggers()
{
    triggers.clear();
    Serial.println("Removed all encoder triggers");
}

bool EncoderTriggerManager::enableTrigger(uint32_t triggerId)
{
    for (auto &trigger : triggers)
    {
        if (trigger.triggerId == triggerId)
        {
            trigger.active = true;
            return true;
        }
    }
    return false;
}

bool EncoderTriggerManager::disableTrigger(uint32_t triggerId)
{
    for (auto &trigger : triggers)
    {
        if (trigger.triggerId == triggerId)
        {
            trigger.active = false;
            return true;
        }
    }
    return false;
}

void EncoderTriggerManager::checkTriggers(float leftDistance, float rightDistance)
{
    std::vector<EncoderTrigger> triggersCopy = triggers;

    for (size_t i = 0; i < triggersCopy.size(); i++)
    {
        auto &trigger = triggersCopy[i];

        if (!trigger.active)
            continue;

        float currentValue = calculateTriggerValue(trigger.type, leftDistance, rightDistance);
        bool conditionMet = checkTriggerCondition(trigger, currentValue);

        if (conditionMet && !trigger.hasTriggered)
        {
            Serial.print("Executing trigger ID: ");
            Serial.print(trigger.triggerId);
            Serial.print(", Current value: ");
            Serial.print(currentValue, 2);
            Serial.print(", Target: ");
            Serial.println(trigger.targetValue, 2);

            for (auto &mainTrigger : triggers)
            {
                if (mainTrigger.triggerId == trigger.triggerId)
                {
                    mainTrigger.hasTriggered = true;

                    try
                    {
                        if (mainTrigger.callback)
                        {
                            mainTrigger.callback(currentValue, mainTrigger.triggerId);
                        }
                    }
                    catch (...)
                    {
                        Serial.print("Exception in trigger callback for ID: ");
                        Serial.println(mainTrigger.triggerId);
                    }

                    if (mainTrigger.oneShot)
                    {
                        removeTrigger(mainTrigger.triggerId);
                    }
                    break;
                }
            }
        }
        else if (!conditionMet && trigger.hasTriggered && !trigger.oneShot)
        {
            for (auto &mainTrigger : triggers)
            {
                if (mainTrigger.triggerId == trigger.triggerId)
                {
                    mainTrigger.hasTriggered = false;
                    break;
                }
            }
        }
    }
}

void EncoderTriggerManager::resetTriggerStates()
{
    for (auto &trigger : triggers)
    {
        trigger.hasTriggered = false;
    }
    Serial.println("Reset all trigger states");
}

bool EncoderTriggerManager::isTriggerActive(uint32_t triggerId)
{
    for (const auto &trigger : triggers)
    {
        if (trigger.triggerId == triggerId)
        {
            return trigger.active;
        }
    }
    return false;
}

uint32_t EncoderTriggerManager::addDistanceTrigger(float distance, std::function<void()> callback, bool oneShot)
{
    return addTrigger(TRIGGER_BOTH_WHEELS, GREATER_EQUAL, distance, [callback](float value, uint32_t id)
                      { 
                         try {
                             callback(); 
                         } catch (...) {
                         } }, oneShot);
}

uint32_t EncoderTriggerManager::addLeftWheelTrigger(float distance, std::function<void()> callback, bool oneShot)
{
    return addTrigger(TRIGGER_LEFT_WHEEL, GREATER_EQUAL, distance, [callback](float value, uint32_t id)
                      { 
                         try {
                             callback(); 
                         } catch (...) {
                         } }, oneShot);
}

uint32_t EncoderTriggerManager::addRightWheelTrigger(float distance, std::function<void()> callback, bool oneShot)
{
    return addTrigger(TRIGGER_RIGHT_WHEEL, GREATER_EQUAL, distance, [callback](float value, uint32_t id)
                      { 
                         try {
                             callback(); 
                         } catch (...) {
                         } }, oneShot);
}

uint32_t EncoderTriggerManager::addRotationTrigger(float wheelBaseDistance, float angleDegrees, std::function<void()> callback, bool oneShot)
{
    float circumference = PI * wheelBaseDistance;
    float wheelDistance = (angleDegrees / 360.0f) * circumference;

    return addTrigger(TRIGGER_LEFT_WHEEL, GREATER_EQUAL, wheelDistance, [callback](float value, uint32_t id)
                      { 
                         try {
                             callback(); 
                         } catch (...) {
                         } }, oneShot);
}
