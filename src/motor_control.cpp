#include "motor_control.h"

MotorControl::MotorControl(int ain1, int ain2, int bin1, int bin2, int stby)
    : pinAin1(ain1), pinAin2(ain2), pinBin1(bin1), pinBin2(bin2), pinSTBY(stby),
      pwmChannelA1(0), pwmChannelA2(1), pwmChannelB1(2), pwmChannelB2(3),
      pwmFrequency(20000), pwmResolution(8),
    currentSpeedA(0), currentSpeedB(0), maxSpeed(255),
    speedBiasA(0), speedBiasB(0),
      kp(2.5), ki(0.00005), kd(0.0009),
      previousError(0.0), integralError(0.0),
      encoder(nullptr), isMoving(false),
      targetDistance(0.0), startLeftDistance(0.0), startRightDistance(0.0)
{
}

void MotorControl::begin(Encoder *encoderRef)
{
    encoder = encoderRef;

    // Configure motor pins as outputs
    pinMode(pinAin1, OUTPUT);
    pinMode(pinAin2, OUTPUT);
    pinMode(pinBin1, OUTPUT);
    pinMode(pinBin2, OUTPUT);
    pinMode(pinSTBY, OUTPUT);

    // DRV8833 doesn't require enable pin, but we use it for power control
    digitalWrite(pinSTBY, HIGH);

    // Setup PWM channels for ESP32 (only need PWM on one pin per motor for DRV8833)
    ledcSetup(pwmChannelA1, pwmFrequency, pwmResolution);
    ledcSetup(pwmChannelA2, pwmFrequency, pwmResolution);
    ledcSetup(pwmChannelB1, pwmFrequency, pwmResolution);
    ledcSetup(pwmChannelB2, pwmFrequency, pwmResolution);

    // Attach pins to PWM channels
    ledcAttachPin(pinAin1, pwmChannelA1);
    ledcAttachPin(pinAin2, pwmChannelA2);
    ledcAttachPin(pinBin1, pwmChannelB1);
    ledcAttachPin(pinBin2, pwmChannelB2);

    // Stop motors
    setMotorSpeed(0, 0);

    Serial.println("Motor control initialized (DRV8833)");
    Serial.println("Motor A (Left): IN1=" + String(pinAin1) + " IN2=" + String(pinAin2));
    Serial.println("Motor B (Right): IN3=" + String(pinBin1) + " IN4=" + String(pinBin2));
    Serial.println("Sleep pin: " + String(pinSTBY));
}

void MotorControl::setMotorASpeed(int speed)
{
    // Constrain speed to valid range
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < -maxSpeed) speed = -maxSpeed;

    if (speed > 0)
    {
        // Forward: IN1 = PWM, IN2 = 0 (motor wiring reversed)
        ledcWrite(pwmChannelA1, speed);
        ledcWrite(pwmChannelA2, 0);
        Serial.print("MotA FWD|IN1(Ch");
        Serial.print(pwmChannelA1);
        Serial.print(")=");
        Serial.print(speed);
        Serial.print("|IN2(Ch");
        Serial.print(pwmChannelA2);
        Serial.println(")=0");
    }
    else if (speed < 0)
    {
        // Backward: IN1 = 0, IN2 = PWM (motor wiring reversed)
        ledcWrite(pwmChannelA1, 0);
        ledcWrite(pwmChannelA2, -speed);
        Serial.print("MotA BWD|IN1(Ch");
        Serial.print(pwmChannelA1);
        Serial.print(")=0|IN2(Ch");
        Serial.print(pwmChannelA2);
        Serial.print(")=");
        Serial.println(-speed);
    }
    else
    {
        // Stop: both 0
        ledcWrite(pwmChannelA1, 0);
        ledcWrite(pwmChannelA2, 0);
        Serial.println("MotA STOP");
    }
    currentSpeedA = speed;
}

void MotorControl::setMotorBSpeed(int speed)
{
    // Constrain speed to valid range
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < -maxSpeed) speed = -maxSpeed;

    if (speed > 0)
    {
        // Forward: IN3 = 0, IN4 = PWM (swapped)
        ledcWrite(pwmChannelB1, 0);
        ledcWrite(pwmChannelB2, speed);
    }
    else if (speed < 0)
    {
        // Backward: IN3 = PWM, IN4 = 0 (swapped)
        ledcWrite(pwmChannelB1, -speed);
        ledcWrite(pwmChannelB2, 0);
    }
    else
    {
        // Stop: both 0
        ledcWrite(pwmChannelB1, 0);
        ledcWrite(pwmChannelB2, 0);
    }
    currentSpeedB = speed;
}

void MotorControl::setMotorSpeed(int speedA, int speedB)
{
    setMotorASpeed(speedA);
    setMotorBSpeed(speedB);
}

void MotorControl::moveForward(int speed)
{
    if (speed > maxSpeed)
        speed = maxSpeed;
    if (speed < 0)
        speed = 0;

    // Apply optional per-motor bias to correct hardware drift
    int speedA = speed + speedBiasA;
    int speedB = speed + speedBiasB;

    setMotorSpeed(speedA, speedB);
    isMoving = true;  // Enable PID control
}

void MotorControl::moveBackward(int speed)
{
    if (speed > maxSpeed)
        speed = maxSpeed;
    if (speed < 0)
        speed = 0;

    setMotorSpeed(-speed, -speed);
    Serial.println("Moving backward at speed: " + String(speed));
}

void MotorControl::turnLeft(int speed)
{
    if (speed > maxSpeed)
        speed = maxSpeed;
    if (speed < 0)
        speed = 0;

    setMotorSpeed(speed / 2, speed);
    Serial.println("Turning left at speed: " + String(speed));
}

void MotorControl::turnRight(int speed)
{
    if (speed > maxSpeed)
        speed = maxSpeed;
    if (speed < 0)
        speed = 0;

    setMotorSpeed(speed, speed / 2);
    Serial.println("Turning right at speed: " + String(speed));
}

void MotorControl::stop()
{
    setMotorSpeed(0, 0);
    isMoving = false;
    Serial.println("Motors stopped");
}

void MotorControl::standby(bool enable)
{
    digitalWrite(pinSTBY, enable ? HIGH : LOW);
    if (enable)
    {
        Serial.println("Motors enabled (standby OFF)");
    }
    else
    {
        Serial.println("Motors disabled (standby ON)");
    }
}

void MotorControl::setPIDCoefficients(float kp_val, float ki_val, float kd_val)
{
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;
    Serial.print("PID coefficients set: Kp=");
    Serial.print(kp);
    Serial.print(" Ki=");
    Serial.print(ki);
    Serial.print(" Kd=");
    Serial.println(kd);
}

void MotorControl::updateStraightLineControl()
{
    if (!encoder || !isMoving)
        return;

    // Get current pulse counts
    long leftCount = encoder->getPulseCount1();
    long rightCount = encoder->getPulseCount2();

    // Calculate error (difference between left and right pulse counts)
    float error = leftCount - rightCount + 0  ;

    // PID calculation
    integralError += error;
    
    // Anti-windup: limit integral
    if (integralError > 500) integralError = 500;
    if (integralError < -500) integralError = -500;
    
    float derivative = error - previousError;
    previousError = error;

    float correction = (kp * error) + (ki * integralError) + (kd * derivative);

    // Apply correction to motor speeds
    int correctedSpeedA = currentSpeedA - (int)correction;
    int correctedSpeedB = currentSpeedB + (int)correction;

    // Constrain speeds
    if (correctedSpeedA > maxSpeed)
        correctedSpeedA = maxSpeed;
    if (correctedSpeedA < -maxSpeed)
        correctedSpeedA = -maxSpeed;
    if (correctedSpeedB > maxSpeed)
        correctedSpeedB = maxSpeed;
    if (correctedSpeedB < -maxSpeed)
        correctedSpeedB = -maxSpeed;

    setMotorSpeed(correctedSpeedA, correctedSpeedB);
}

void MotorControl::moveForwardDistance(float distance, int baseSpeed)
{
    if (!encoder)
    {
        Serial.println("Error: Encoder not initialized");
        return;
    }

    if (distance < 0)
    {
        moveBackwardDistance(-distance, baseSpeed);
        return;
    }

    if (baseSpeed > maxSpeed)
        baseSpeed = maxSpeed;
    if (baseSpeed < 0)
        baseSpeed = 0;

    // Reset encoder readings
    encoder->reset(true);
    startLeftDistance = 0;
    startRightDistance = 0;

    // Initialize PID control
    previousError = 0;
    integralError = 0;

    // Set target distance
    targetDistance = distance;
    isMoving = true;

    Serial.println("Moving forward " + String(distance, 4) + " meters at base speed " + String(baseSpeed));

    // Movement loop
    while (isMoving)
    {
        // Get current distances
        float leftDistance = encoder->getDistance1();
        float rightDistance = encoder->getDistance2();
        float avgDistance = (abs(leftDistance) + abs(rightDistance)) / 2.0f;

        // Update straight line control
        moveForward(baseSpeed);  // Set base speed
        updateStraightLineControl();  // Apply PID correction

        // Print progress
        Serial.print("Distance: ");
        Serial.print(avgDistance, 4);
        Serial.print(" m | Left: ");
        Serial.print(leftDistance, 4);
        Serial.print(" m | Right: ");
        Serial.print(rightDistance, 4);
        Serial.println(" m");

        // Check if target distance reached
        if (avgDistance >= targetDistance)
        {
            stop();
            Serial.println("Target distance reached!");
            break;
        }

        delay(50);  // Update control every 50ms
    }
}

void MotorControl::moveBackwardDistance(float distance, int baseSpeed)
{
    if (!encoder)
    {
        Serial.println("Error: Encoder not initialized");
        return;
    }

    if (distance < 0)
    {
        moveForwardDistance(-distance, baseSpeed);
        return;
    }

    if (baseSpeed > maxSpeed)
        baseSpeed = maxSpeed;
    if (baseSpeed < 0)
        baseSpeed = 0;

    // Reset encoder readings
    encoder->reset(true);
    startLeftDistance = 0;
    startRightDistance = 0;

    // Initialize PID control
    previousError = 0;
    integralError = 0;

    // Set target distance
    targetDistance = distance;
    isMoving = true;

    Serial.println("Moving backward " + String(distance, 4) + " meters at base speed " + String(baseSpeed));

    // Movement loop
    while (isMoving)
    {
        // Get current distances
        float leftDistance = encoder->getDistance1();
        float rightDistance = encoder->getDistance2();
        float avgDistance = (abs(leftDistance) + abs(rightDistance)) / 2.0f;

        // Update straight line control
        moveBackward(baseSpeed);  // Set base speed
        updateStraightLineControl();  // Apply PID correction

        // Print progress
        Serial.print("Distance: ");
        Serial.print(avgDistance, 4);
        Serial.print(" m | Left: ");
        Serial.print(leftDistance, 4);
        Serial.print(" m | Right: ");
        Serial.print(rightDistance, 4);
        Serial.println(" m");

        // Check if target distance reached
        if (avgDistance >= targetDistance)
        {
            stop();
            Serial.println("Target distance reached!");
            break;
        }

        delay(50);  // Update control every 50ms
    }
}

int MotorControl::getCurrentSpeedA() const
{
    return currentSpeedA;
}

int MotorControl::getCurrentSpeedB() const
{
    return currentSpeedB;
}

bool MotorControl::isMovingForward() const
{
    return isMoving && currentSpeedA > 0 && currentSpeedB > 0;
}

float MotorControl::getTargetDistance() const
{
    return targetDistance;
}
void MotorControl::setSpeedBias(int biasA, int biasB)
{
    speedBiasA = biasA;
    speedBiasB = biasB;
    Serial.print("Speed bias set - Left: ");
    Serial.print(biasA);
    Serial.print(" | Right: ");
    Serial.println(biasB);
}

void MotorControl::setLeftMotorBias(int bias)
{
    speedBiasA = bias;
    Serial.print("Left motor bias set to: ");
    Serial.println(bias);
}

void MotorControl::setRightMotorBias(int bias)
{
    speedBiasB = bias;
    Serial.print("Right motor bias set to: ");
    Serial.println(bias);
}

int MotorControl::getLeftMotorBias() const
{
    return speedBiasA;
}

int MotorControl::getRightMotorBias() const
{
    return speedBiasB;
}