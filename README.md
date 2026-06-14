# Micromouse - Autonomous Maze-Solving Robot

An autonomous robotic system that navigates and solves mazes using real-time sensor feedback, advanced control algorithms, and intelligent path planning. The robot dynamically discovers the maze layout and efficiently finds the shortest path to the center.

## Overview

This Micromouse implementation features a fully autonomous maze-solving robot built on the **ESP32** microcontroller. The robot uses a combination of Time-of-Flight sensors for wall detection, gyroscopic feedback for orientation control, and encoder-based distance tracking to navigate through a 16×16 cell maze with precision.
## Gallery

A selection of Micromouse photos (hosted in the repository):

<p align="center">
   <img src="https://raw.githubusercontent.com/gunasekaraks/Micromouse/main/WhatsApp%20Image%202026-06-14%20at%2019.38.53.jpeg" alt="Micromouse front view" width="320" />
   <img src="https://raw.githubusercontent.com/gunasekaraks/Micromouse/main/WhatsApp%20Image%202026-06-14%20at%2019.39.18%20(1).jpeg" alt="Micromouse top view" width="320" />
   <img src="https://raw.githubusercontent.com/gunasekaraks/Micromouse/main/WhatsApp%20Image%202026-06-14%20at%2019.39.18.jpeg" alt="Micromouse test setup" width="320" />
</p>



### Key Capabilities

- **Real-time Wall Detection**: Three VL53L0X Time-of-Flight (ToF) sensors for front, left, and right obstacle detection
- **Precise Motion Control**: PID-based motor control with encoder feedback for straight-line movement and accurate turns
- **Intelligent Navigation**: Flood-fill search algorithm with A* pathfinding for optimal route discovery
- **Orientation Tracking**: MPU6050 gyroscope integration for accurate heading alignment and drift correction
- **Wireless Telemetry**: WiFi connectivity for real-time telemetry and remote monitoring
- **Autonomous Maze Solving**: Fully autonomous from initialization to center detection

## Hardware Requirements

### Microcontroller
- **ESP32** development board

### Sensors
- **2x DC Motors** with encoders (left and right wheels)
- **3x VL53L0X Time-of-Flight Sensors** (front, left, right)
- **MPU6050** 6-axis IMU (gyroscope + accelerometer)
- **Buzzer** for audio feedback

### Motor Drive
- **TB6612FNG** or similar motor driver IC for dual DC motor control

### Power
- Li-Po battery or equivalent power supply
- Motor power supply (separate from logic supply recommended)

## Software Architecture

### Core Modules

#### 1. **Encoder Module** (`encoder.h/cpp`)
- Quadrature encoder reading for both wheels
- Supports different PPR (Pulses Per Revolution) for left (357 PPR) and right (207 PPR) wheels
- Distance calculation and trigger system for event-based control
- ISR-based pulse counting for real-time tracking

#### 2. **Motor Control** (`motor_control.h/cpp`)
- PWM-based speed control for both motors
- PID-based straight-line correction using encoder feedback
- Distance-based movement primitives
- Motor bias calibration for drift compensation

#### 3. **Gyroscope Module** (`gyro.h/cpp`)
- MPU6050 initialization and configuration
- DMP (Digital Motion Processor) for 6-axis fusion
- Yaw angle tracking and drift management
- Interrupt-driven data acquisition

#### 4. **ToF Sensors** (`vl53l0x_v2.h/cpp`)
- Multi-sensor management for front, left, and right walls
- Median filtering for noise rejection
- Outlier rejection to handle sensor spikes
- Real-time distance measurements in millimeters

#### 5. **Turn Control** (`turn.h/cpp`)
- Precise 90°, 180° rotation using encoder feedback
- Gyro-assisted turn validation
- Cardinal heading alignment

#### 6. **Search Algorithm** (`search_run.h/cpp`)
The core maze-solving engine featuring:
- **Flood-fill algorithm** with A* pathfinding
- **Dynamic maze mapping** (16×16 grid)
- **Wall tracking** using 4-bit encoding (North, East, South, West)
- **Center detection** (2×2 open area in maze center)
- **Intelligent navigation** prioritizing unexplored cells over backtracking
- **ToF-based edge validation** before committing to moves

#### 7. **WiFi Management** (`wifi_manager.h/cpp`)
- Real-time telemetry transmission
- Maze state and movement logging
- Status monitoring and debugging interface

## Algorithm Details

### Maze Navigation Strategy

1. **Initialization Phase**
   - Robot starts at grid center (7,7) facing North
   - Calibrates sensors and stabilizes gyroscope
   - Initializes 16×16 wall grid and visited cell tracking

2. **Exploration Phase**
   - Uses **flood-fill search** to detect the 2×2 maze center
   - Prioritizes unexplored cells over backtracking
   - Validates wall edges using ToF sensors before moving
   - Updates wall map in real-time as maze layout is discovered

3. **Path Planning**
   - **A* algorithm** computes optimal path to detected center
   - Manhattan distance heuristic for maze-based navigation
   - Recomputes path when new walls are discovered
   - Handles backtracking intelligently when hitting dead ends

4. **Movement Execution**
   - Orients robot to next direction (0-3 turns maximum)
   - Validates path is clear using front ToF sensor
   - Executes forward movement with PID-based straight-line control
   - Corrects yaw drift using gyroscope feedback post-movement

### Maze Representation

```
walls[y][x] - 8-bit value encoding walls in 4 directions:
- Bit 0: NORTH wall
- Bit 1: EAST wall  
- Bit 2: SOUTH wall
- Bit 3: WEST wall
```

### Sensor Thresholds

```cpp
FRONT_THRESH_MM = 90mm   // Front wall detection (18cm cell)
SIDE_THRESH_MM = 90mm    // Side wall detection (16.8cm corridor)
```

## Pin Configuration

### ESP32 Pin Assignments

| Function | Pin | Type |
|----------|-----|------|
| **Motor A (Left)** | | |
| A_IN1 | 25 | PWM |
| A_IN2 | 13 | PWM |
| **Motor B (Right)** | | |
| B_IN1 | 14 | PWM |
| B_IN2 | 18 | PWM |
| Standby (STBY) | 32 | GPIO |
| **Encoders** | | |
| Left A | 35 | GPIO (Interrupt) |
| Left B | 34 | GPIO (Interrupt) |
| Right A | 19 | GPIO (Interrupt) |
| Right B | 26 | GPIO (Interrupt) |
| **I2C Bus** | | |
| SDA | 21 | I2C |
| SCL | 22 | I2C |
| **Gyro Interrupt** | 33 | GPIO (Interrupt) |
| **Buzzer** | 12 | GPIO |

## Building & Deployment

### Prerequisites
- **PlatformIO** IDE or CLI
- **Arduino Framework** for ESP32
- Required libraries (defined in `platformio.ini`)

### Compilation
```bash
platformio run -e esp32dev
```

### Upload to ESP32
```bash
platformio run -e esp32dev --target upload
```

### Serial Monitoring
```bash
platformio device monitor --baud 115200
```

## Telemetry Output

The robot streams real-time telemetry over Serial (115200 baud) and WiFi:

```
SearchRun|Cell(7,7)|Dir:N->E|Dist:3|ToF_F:250.5mm|R:240.2mm|L:235.8mm
L:1234|R:1200|Err:34|L_Speed:180|R_Speed:185|AvgDist_cm:12.5
Turn|L:357/400|R:207/400|LSpd:150|RSpd:-150|AvgDist_m:0.1234
CENTER_FOUND|Goal at (7,7) to (8,8)
SUCCESS|Reached center at (7,7) in 42 steps
```

## Calibration

### Motor Bias Tuning
Adjust the right motor bias to compensate for mechanical drift:
```cpp
motorControl.setRightMotorBias(28);  // Positive = increase right speed
```

### PID Coefficients
Fine-tune straight-line control in `main.cpp`:
```cpp
motorControl.setPIDCoefficients(2.5f, 0.0005f, 0.00005f);  // Kp, Ki, Kd
```

### Wheel Configuration
Update in encoder initialization:
```cpp
Encoder encoder(35, 34, 19, 26, 
    wheelCircumference, 
    357,  // Left PPR
    357   // Right PPR
);
```

## Testing

### Movement Test
Run forward movement test to validate encoder and motor control:
- **File**: `src/moveforward.cpp`
- Moves robot 18cm forward with gyro-assisted alignment

### Turn Test
Enable turn validation:
```cpp
#define RUN_TURN_TEST 1
```
Tests 90° right turn, pause, 90° left turn sequence.

### Serial Debugging
Monitor real-time debug output through Serial:
- Motor speeds and encoder counts
- ToF sensor readings
- Gyroscope yaw values
- Maze state updates

## Known Limitations & Future Enhancements

### Current Limitations
- Fixed 16×16 maze grid (adjustable in code)
- Requires WiFi for full telemetry (Serial-only mode available)
- Assumes standard maze cell size (18cm)

### Potential Enhancements
- **Fast Run**: Optimize path after initial exploration
- **Speed Optimization**: Increase velocity during known sections
- **Multi-maze Support**: Adaptive grid sizing
- **IMU Calibration**: Automatic gyro bias correction
- **Advanced Filtering**: Kalman filtering for sensor fusion

## Troubleshooting

### Robot Drifts Right/Left
- Adjust motor bias: `motorControl.setRightMotorBias(value)`
- Check wheel circumference values
- Verify encoder PPR settings match hardware

### ToF Sensors Not Detecting Walls
- Verify I2C connections and addresses
- Check sensor orientation (straight forward)
- Adjust thresholds (`FRONT_THRESH_MM`, `SIDE_THRESH_MM`)

### Gyro Not Stabilizing
- Wait longer in `setup()` for gyro to calibrate
- Ensure robot is stationary during initialization
- Check MPU6050 interrupt pin connection

### Turns Not Precise
- Verify wheel base distance (8cm default)
- Check encoder calibration
- Adjust PID coefficients iteratively

## Project Structure

```
Micromouse/
├── include/               # Header files
│   ├── encoder.h         # Encoder driver
│   ├── motor_control.h   # Motor control system
│   ├── gyro.h            # Gyroscope driver
│   ├── vl53l0x_v2.h      # ToF sensor driver
│   ├── turn.h            # Turn control
│   ├── search_run.h      # Maze solving algorithm
│   └── wifi_manager.h    # WiFi telemetry
├── src/                   # Implementation files
│   ├── main.cpp          # Entry point
│   ├── encoder.cpp
│   ├── motor_control.cpp
│   ├── gyro.cpp
│   ├── vl53l0x_v2.cpp
│   ├── turn.cpp
│   ├── search_run.cpp
│   ├── moveforward.cpp   # Forward movement test
│   ├── wifi_manager.cpp
│   ├── encoder_trigger.cpp
│   └── buzzer.cpp        # Audio feedback
├── platformio.ini        # Build configuration
└── README.md            # This file
```

## References & Resources

### Micromouse Competition
- [IEEE Micromouse Official](https://www.ieee.org/)
- Standard 16×16 cell maze format
- Typical cell size: 180mm

### Hardware Libraries Used
- **VL53L0X** - Time-of-Flight sensor library
- **MPU6050** - IMU library with DMP support
- **Arduino Core** - ESP32 Arduino framework

### Algorithms
- **Flood Fill** - Classic maze exploration
- **A* Pathfinding** - Optimal path planning
- **PID Control** - Motor speed regulation

## License

This project is open source. Use at your own discretion for educational and competitive purposes.

## Contributors

- **Kavin Sulakshitha Gunasekara**
- **Amiru Munasinghe**

## Contact & Support

For questions, issues, or contributions, please refer to the GitHub repository:
[gunasekaraks/Micromouse](https://github.com/gunasekaraks/Micromouse)

---

**Last Updated**: June 2026  
**Version**: 1.0  
**Status**: Active Development
