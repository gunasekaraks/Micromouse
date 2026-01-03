#include "encoder.h"
#include "encoder_trigger.h"

// Static instance pointer for ISR access
Encoder *Encoder::instance = nullptr;
float distancePerPulse1;
float distancePerPulse2;

Encoder::Encoder(int pinA1, int pinB1, int pinA2, int pinB2, float wheelCirc, int ppr1, int ppr2)
    : encoderPinA1(pinA1), encoderPinB1(pinB1), encoderPinA2(pinA2), encoderPinB2(pinB2),
      wheelCircumference(wheelCirc), pulsesPerRevolution1(ppr1), pulsesPerRevolution2(ppr2),
      pulseCount1(0), pulseCount2(0), totalDistance1(0.0), totalDistance2(0.0),
      pulseDetected1(false), pulseDetected2(false), lastDirection1(0), lastDirection2(0)
{
    instance = this; // Set static instance for ISR access
    distancePerPulse1 = wheelCircumference / pulsesPerRevolution1;
    distancePerPulse2 = wheelCircumference / pulsesPerRevolution2;
}

void Encoder::begin()
{
    pinMode(encoderPinA1, INPUT_PULLUP);
    pinMode(encoderPinB1, INPUT_PULLUP);
    pinMode(encoderPinA2, INPUT_PULLUP);
    pinMode(encoderPinB2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encoderPinA1), encoderISR1, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA2), encoderISR2, RISING);

    Serial.println("Encoders initialized with pins:");
    Serial.println("Left Motor - A1:" + String(encoderPinA1) + " B1:" + String(encoderPinB1));
    Serial.println("Right Motor - A2:" + String(encoderPinA2) + " B2:" + String(encoderPinB2));
}

void Encoder::reset(bool keepTriggers)
{
    pulseCount1 = 0;
    pulseCount2 = 0;
    totalDistance1 = 0.0;
    totalDistance2 = 0.0;
    pulseDetected1 = false;
    pulseDetected2 = false;
    lastDirection1 = 0;
    lastDirection2 = 0;

    Serial.println("Encoders reset");
}

void IRAM_ATTR Encoder::encoderISR1()
{
    if (instance)
    {
        instance->handleEncoder1();
    }
}

void IRAM_ATTR Encoder::encoderISR2()
{
    if (instance)
    {
        instance->handleEncoder2();
    }
}

void Encoder::handleEncoder1()
{
    int b = digitalRead(encoderPinB1);
    if (b == LOW)  // Inverted for left motor
    {
        pulseCount1++;
        lastDirection1 = 1;
    }
    else
    {
        pulseCount1--;
        lastDirection1 = -1;
    }
    pulseDetected1 = true;
}

void Encoder::handleEncoder2()
{
    int b = digitalRead(encoderPinB2);
    if (b == LOW)  // Inverted for right motor
    {
        pulseCount2++;
        lastDirection2 = 1;
    }
    else
    {
        pulseCount2--;
        lastDirection2 = -1;
    }
    pulseDetected2 = true;
}

float Encoder::getDistance1() const
{
    return pulseCount1 * distancePerPulse1;
}

float Encoder::getDistance2() const
{
    return pulseCount2 * distancePerPulse2;
}

float Encoder::getResultantDistance() const
{
    return (abs(getDistance1()) + abs(getDistance2())) / 2.0f;
}

long Encoder::getPulseCount1() const
{
    return pulseCount1;
}

long Encoder::getPulseCount2() const
{
    return pulseCount2;
}

int Encoder::getDirection1() const
{
    return lastDirection1;
}

int Encoder::getDirection2() const
{
    return lastDirection2;
}

uint32_t Encoder::addTrigger(TriggerType type, TriggerCondition condition, float targetValue,
                              std::function<void(float, uint32_t)> callback, bool oneShot)
{
    // This would use the EncoderTriggerManager - implementation would be added if needed
    return 0;
}

void Encoder::checkTriggers(float leftDistance, float rightDistance)
{
    // Trigger checking logic would go here
}

bool Encoder::removeTrigger(uint32_t triggerId)
{
    return false;
}

void Encoder::removeAllTriggers()
{
}

bool Encoder::enableTrigger(uint32_t triggerId)
{
    return false;
}

bool Encoder::disableTrigger(uint32_t triggerId)
{
    return false;
}

uint32_t Encoder::addDistanceTrigger(float distance, std::function<void()> callback, bool oneShot)
{
    return 0;
}

uint32_t Encoder::addLeftWheelTrigger(float distance, std::function<void()> callback, bool oneShot)
{
    return 0;
}

uint32_t Encoder::addRightWheelTrigger(float distance, std::function<void()> callback, bool oneShot)
{
    return 0;
}

uint32_t Encoder::addRotationTrigger(float wheelBaseDistance, float angleDegrees, std::function<void()> callback, bool oneShot)
{
    return 0;
}
