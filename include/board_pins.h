#pragma once
#include <Arduino.h>

namespace BoardPins
{
    // Board/PCB mapping (keep all GPIO assignments here)
    constexpr gpio_num_t HEARTBEAT_LED = GPIO_NUM_2;

    // Left motor H-bridge (dual PWM inputs)
    constexpr gpio_num_t LEFT_MOTOR_IN1 = GPIO_NUM_25;
    constexpr gpio_num_t LEFT_MOTOR_IN2 = GPIO_NUM_26;
}
