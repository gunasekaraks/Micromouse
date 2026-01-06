#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "encoder.h"

class MotorControl
{
private:
    // Motor A (Left Motor) pins
    int pinAin1;  // D25
    int pinAin2;  // D13

    // Motor B (Right Motor) pins
    int pinBin1;  // D14
    int pinBin2;  // D18

    // Standby pin
    int pinSTBY;  // D32

    // PWM channels for ESP32
    int pwmChannelA1, pwmChannelA2;
    int pwmChannelB1, pwmChannelB2;
    int pwmFrequency;
    int pwmResolution;

    // Motor speed control
    int currentSpeedA;
    int currentSpeedB;
    int maxSpeed;
    int speedBiasA;  // Per-motor bias to correct drift (left)
    int speedBiasB;  // Per-motor bias to correct drift (right)

    // PID control for straight line movement
    float kp, ki, kd;  // PID coefficients
    float previousError;
    float integralError;

    // Reference to encoder
    Encoder *encoder;

    // Movement state
    bool isMoving;
    float targetDistance;
    float startLeftDistance;
    float startRightDistance;

public:
    // Constructor
    MotorControl(int ain1 = 25, int ain2 = 13, int bin1 = 14, int bin2 = 18, int stby = 32);

    // Initialization
    void begin(Encoder *encoderRef);

    // Motor control methods
    void moveForward(int speed = 200);
    void moveBackward(int speed = 200);
    void turnLeft(int speed = 150);
    void turnRight(int speed = 150);
    void stop();
    void standby(bool enable);

    // Distance-based movement with straight line correction
    void moveForwardDistance(float distance, int baseSpeed = 200);
    void moveBackwardDistance(float distance, int baseSpeed = 200);

    // PID straight line control
    void setPIDCoefficients(float kp, float ki, float kd);
    void updateStraightLineControl();

    // Getters
    int getCurrentSpeedA() const;
    int getCurrentSpeedB() const;
    bool isMovingForward() const;
    float getTargetDistance() const;

    // Direct motor control (public for custom speed control)
    void setMotorASpeed(int speed);  // -255 to 255
    void setMotorBSpeed(int speed);  // -255 to 255

private:
    // Internal motor control
    void setMotorSpeed(int speedA, int speedB);
};

#endif
