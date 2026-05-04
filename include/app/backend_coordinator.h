#pragma once

#include <Arduino.h>

#include "app/drive_controller.h"
#include "app/navigation_controller.h"
#include "app/led_controller.h"
#include "app/robot_state.h"
#include "app/sensor_suite.h"
#include "drivers/i2s_audio.h"

class BackendCoordinator
{
public:
    BackendCoordinator(RobotState &state, DriveController &drive, SensorSuite &sensors, NavigationController &navigation, LedController &leds, I2sAudio &audio);

    void begin();
    void handle();

    void registerTask(uint32_t nowMs);
    void stateTask(uint32_t nowMs);
    void pushState();

    void driveTask(uint32_t nowMs);

private:
    RobotState &state;
    DriveController &drive;
    SensorSuite &sensors;
    NavigationController &navigation;
    LedController &leds;
    I2sAudio &audio;

    uint32_t lastBackendRegisterMs;
    uint32_t lastBackendStateMs;

    bool stateDirty;
    bool stateUrgent;
    bool wsSeenConnected;
    bool wsDisconnectAlerted;
    uint32_t wsDisconnectedSinceMs;
    float manualSpeedCapMultiplier;
};
