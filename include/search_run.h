#pragma once

#include <Arduino.h>

class WiFiManager;
class Encoder;
class MotorControl;

namespace SearchRun {

// Cardinal directions
enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// Initialize the search module; inject dependencies
// Optional startX, startY, startDir allow variable starting position (default: center 7,7 facing North)
void begin(WiFiManager* wifi, Encoder* enc, MotorControl* motors, int startX = 7, int startY = 7, Dir startDir = NORTH);

// Run flood-fill search to center; blocks until one of the center cells is reached
void run();

} // namespace SearchRun
