#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <Arduino.h>

// Current drive mode of the robot
enum DriveMode
{
    IDLE,
    MANUAL,
    AUTO
};

struct SystemState
{
    // Telemetry data
    float lux = 0.0f;
    bool obstacleLeft = false;
    bool obstacleRight = false;
    int batteryLevel = 100; // in percent

    // Status info
    DriveMode driveMode = IDLE;
    String systemHealth = "OK";
    String currentPosition = "Unknown";

    // Timing
    unsigned long lastStatusUpdate = 0;
};

#endif