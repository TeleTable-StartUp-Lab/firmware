#include "core/system_state.h"

SystemState::SystemState()
{
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

    _mtx = xSemaphoreCreateMutex();
}

void SystemState::lock()
{
    if (_mtx)
        xSemaphoreTake(_mtx, portMAX_DELAY);
}

void SystemState::unlock()
{
    if (_mtx)
        xSemaphoreGive(_mtx);
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
