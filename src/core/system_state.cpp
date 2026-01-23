#include "system_state.h"

SystemState::SystemState()
{
    // Default values
    lux = 0.0f;
    obstacleLeft = false;
    obstacleRight = false;
    batteryLevel = 100;

    driveMode = IDLE;
    systemHealth = "Initializing...";
    currentPosition = "Docking Station";

    lastStatusUpdate = 0;
    linearVelocity = 0.0f;
    angularVelocity = 0.0f;
}

const char *SystemState::driveModeToCString() const
{
    switch (driveMode)
    {
    case MANUAL:
        return "MANUAL";
    case AUTO:
        return "AUTO";
    case IDLE:
    default:
        return "IDLE";
    }
}

bool SystemState::setDriveModeFromString(const String &mode)
{
    if (mode == "MANUAL")
    {
        driveMode = MANUAL;
        return true;
    }
    if (mode == "AUTO")
    {
        driveMode = AUTO;
        return true;
    }
    if (mode == "IDLE")
    {
        driveMode = IDLE;
        return true;
    }
    return false;
}
