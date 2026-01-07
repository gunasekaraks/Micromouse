#ifndef TURN_H
#define TURN_H

#include <Arduino.h>

// Turn direction constants (positive = right/CW, negative = left/CCW)
#define TURN_RIGHT 90
#define TURN_LEFT -90
#define TURN_180 180

// Turn the robot by specified angle using encoders and gyro feedback
// angle: target angle in degrees (positive = left/CCW, negative = right/CW)
// Returns true if successful, false if timeout or error
bool turn(float angle);

// Convenience functions for common turns
bool turnRight90();
bool turnLeft90();
bool turn180();

// Encoder-based turn with PID and gyro feedback for precise rotation
// More accurate than gyro-only turning
bool turnEncoderBased(float angle);

#endif // TURN_H
