#pragma once

#include <Arduino.h>

#include "app/drive_controller.h"
#include "app/led_controller.h"
#include "app/sensor_suite.h"
#include "app/serial_console.h"
#include "drivers/i2s_audio.h"

class ConsoleCommander
{
public:
    ConsoleCommander(DriveController &drive, SensorSuite &sensors, LedController &leds, I2sAudio &audio);

    void begin();
    void handle();
    void printHelp();

private:
    SerialConsole console;
    DriveController &drive;
    SensorSuite &sensors;
    LedController &leds;
    I2sAudio &audio;
};
