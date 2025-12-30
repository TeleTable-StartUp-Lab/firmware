#include "system_state.h"

SystemState::SystemState()
{
    // Initializing default values to ensure predictable behavior
    lux = 0.0f;
    obstacleLeft = false;
    obstacleRight = false;
    batteryLevel = 100;

    driveMode = IDLE;
    systemHealth = "Initializing...";
    currentPosition = "Docking Station";

    lastStatusUpdate = 0;
}