#pragma once

#include <Arduino.h>

namespace AppConfig
{
    constexpr uint32_t SERIAL_BAUD = 115200;

    constexpr gpio_num_t HEARTBEAT_LED = GPIO_NUM_2;

    constexpr uint32_t HEARTBEAT_PERIOD_MS = 500;
    constexpr uint32_t STATUS_PRINT_PERIOD_MS = 1000;
}
