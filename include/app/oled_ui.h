#pragma once

#include <Arduino.h>

#include "app/robot_state.h"
#include "app/sensor_suite.h"
#include "drivers/oled_display.h"

class OledUi
{
public:
    OledUi(RobotState &state, SensorSuite &sensors);

    bool begin();
    void bootScreen();
    void update(uint32_t nowMs);

    bool isOk() const;
    bool probe() const;
    bool recover();
    void clear();
    void showTestScreen();

private:
    OledDisplay oled;
    RobotState &state;
    SensorSuite &sensors;
    uint32_t lastOledMs;
    uint32_t lastRecoveryAttemptMs;
};
