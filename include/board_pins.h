#pragma once
#include <Arduino.h>

namespace BoardPins
{
    constexpr gpio_num_t HEARTBEAT_LED = GPIO_NUM_2;

    constexpr gpio_num_t LEFT_MOTOR_IN1 = GPIO_NUM_25;
    constexpr gpio_num_t LEFT_MOTOR_IN2 = GPIO_NUM_26;

    constexpr gpio_num_t RIGHT_MOTOR_IN1 = GPIO_NUM_27;
    constexpr gpio_num_t RIGHT_MOTOR_IN2 = GPIO_NUM_14;
}
