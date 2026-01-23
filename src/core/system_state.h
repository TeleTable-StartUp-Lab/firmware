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
    // Constructor
    SystemState();

    // Telemetry data
    float lux;
    bool obstacleLeft;
    bool obstacleRight;
    int batteryLevel; // in percent

    // Status info
    DriveMode driveMode;
    String systemHealth;
    String currentPosition;

    // Timing
    unsigned long lastStatusUpdate;

    // Helpers
    const char *driveModeToCString() const;
    bool setDriveModeFromString(const String &mode);
};

#endif
