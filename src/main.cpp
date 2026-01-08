#include <Arduino.h>
#include <Wire.h>
#include "wifi_manager.h"
#include "vl53l0x_v2.h"
#include "encoder.h"
#include "motor_control.h"
#include "gyro.h"
#include "search_run.h"
#include "turn.h"

// Set to 1 to run a simple 90° turn test instead of the search run
#define RUN_TURN_TEST 1

// Global instances (needed by turn.cpp as well)
WiFiManager wifiMgr;

// Wheel configuration
static constexpr float wheelDiameterMeters = 0.034f;
static constexpr float wheelCircumferenceMeters = wheelDiameterMeters * 3.14159f;

// Encoder and motor control
Encoder encoder(35, 34, 19, 26, wheelCircumferenceMeters, 357, 357);
MotorControl motorControl(25, 13, 14, 18, 32);

// Simple turn test: execute a right 90°, pause, then a left 90°
static void runTurnTest()
{
    Serial.println("\n=== TURN TEST MODE ===");
    wifiMgr.sendDataLn("TURN_TEST|Starting");

    // Ensure encoders start from zero
    encoder.reset(true);

    delay(500);
    Serial.println("Right 90° turn...");
    bool ok = turnRight90();
    Serial.println(ok ? "Right turn complete" : "Right turn failed");
    wifiMgr.sendDataLn(ok ? "TURN_TEST|Right90|OK" : "TURN_TEST|Right90|FAIL");

    delay(1500);

    Serial.println("Left 90° turn (return to heading)...");
    ok = turnLeft90();
    Serial.println(ok ? "Left turn complete" : "Left turn failed");
    wifiMgr.sendDataLn(ok ? "TURN_TEST|Left90|OK" : "TURN_TEST|Left90|FAIL");

    Serial.println("=== TURN TEST DONE ===");
    wifiMgr.sendDataLn("TURN_TEST|Done");
}

void setup()
{
    Serial.begin(115200);
    delay(200);

    // Basic init required for both turn test and search
    wifiMgr.begin();
    encoder.begin();
    motorControl.begin(&encoder);
    motorControl.setPIDCoefficients(2.5f, 0.0005f, 0.00005f);

#if RUN_TURN_TEST
    // Exercise the turn routine with PWM initialized
    runTurnTest();
    Serial.println("Hold here after turn test.");
    while (1) {
        wifiMgr.update();
        delay(500);
    }
#endif

    // Initialize ToF sensors
    Serial.println("Initializing ToF sensors...");
    bool tofOk = setupToF();
    if (!tofOk) {
        Serial.println("ERROR: No ToF sensors initialized; halting.");
        while (1) { delay(1000); }
    }
    flushToF();
    Serial.println("ToF sensors initialized.");
    
    // Verify all ToF sensors are reading
    Serial.println("Verifying ToF sensors...");
    delay(200);
    bool tofReadOk = readToF();
    if (!tofReadOk) {
        Serial.println("ERROR: ToF sensors not responding!");
        while (1) { delay(1000); }
    }
    
    // Check individual sensors
    Serial.println("ToF Readings:");
    Serial.print("  Front: ");
    Serial.print(tof_distance[TOF_FRONT]);
    Serial.print(" mm ");
    if (tof_ready[TOF_FRONT]) Serial.println("✓");
    else Serial.println("✗ NOT READY");
    
    Serial.print("  Right: ");
    Serial.print(tof_distance[TOF_RIGHT]);
    Serial.print(" mm ");
    if (tof_ready[TOF_RIGHT]) Serial.println("✓");
    else Serial.println("✗ NOT READY");
    
    Serial.print("  Left:  ");
    Serial.print(tof_distance[TOF_LEFT]);
    Serial.print(" mm ");
    if (tof_ready[TOF_LEFT]) Serial.println("✓");
    else Serial.println("✗ NOT READY");
    
    // Check if all sensors are ready
    int readySensors = 0;
    if (tof_ready[TOF_FRONT]) readySensors++;
    if (tof_ready[TOF_RIGHT]) readySensors++;
    if (tof_ready[TOF_LEFT]) readySensors++;
    
    Serial.print("Ready sensors: ");
    Serial.print(readySensors);
    Serial.println("/3");
    
    if (readySensors < 3) {
        Serial.println("WARNING: Not all ToF sensors are ready!");
        Serial.println("Continuing anyway, but wall detection may be affected.");
        delay(2000);
    } else {
        Serial.println("✓ All ToF sensors verified and working");
    }

    delay(1000);

    // Initialize search run
    Serial.println("Starting flood fill search run...");
    // IMPORTANT: Always start at center of grid (7,7) regardless of physical corner
    // This allows exploration in all 4 directions without hitting artificial grid boundaries
    // Physical maze walls will constrain movement, not grid bounds
    // Direction can be adjusted based on initial orientation:
    // - North if facing into maze from bottom corner
    // - East if facing into maze from left corner  
    // - South if facing into maze from top corner
    // - West if facing into maze from right corner
    SearchRun::begin(&wifiMgr, &encoder, &motorControl, 7, 7, SearchRun::NORTH);

}

void loop()
{
    // Run the search once
    SearchRun::run();

    // After reaching center, stop
    Serial.println("\n=== SEARCH COMPLETE ===");
    Serial.println("Robot has reached the center. Halting.");
    
    while (1) {
        wifiMgr.update();
        delay(1000);
    }
}




