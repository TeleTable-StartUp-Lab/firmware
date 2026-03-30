#pragma once

#include <Arduino.h>

#include "app/robot_state.h"
#include "drivers/oled_display.h"

class OledUi
{
public:
    explicit OledUi(RobotState &state);

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
    uint32_t lastOledMs;
    uint32_t lastRecoveryAttemptMs;
};
