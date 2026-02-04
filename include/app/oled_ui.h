#pragma once

#include <Arduino.h>

#include "app/drive_controller.h"
#include "app/led_controller.h"
#include "app/sensor_suite.h"
#include "drivers/oled_display.h"

class OledUi
{
public:
    OledUi(SensorSuite &sensors, LedController &leds, DriveController &drive);

    bool begin();
    void bootScreen();
    void update(uint32_t nowMs);

    bool isOk() const;

private:
    OledDisplay oled;
    SensorSuite &sensors;
    LedController &leds;
    DriveController &drive;
    uint32_t lastOledMs;
};
