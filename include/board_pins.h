#pragma once
#include <Arduino.h>

namespace BoardPins
{

    constexpr gpio_num_t LEFT_MOTOR_IN1 = GPIO_NUM_25;
    constexpr gpio_num_t LEFT_MOTOR_IN2 = GPIO_NUM_26;

    constexpr gpio_num_t RIGHT_MOTOR_IN1 = GPIO_NUM_15;
    constexpr gpio_num_t RIGHT_MOTOR_IN2 = GPIO_NUM_2;

    constexpr gpio_num_t IR_LEFT = GPIO_NUM_32;
    constexpr gpio_num_t IR_MIDDLE = GPIO_NUM_33;
    constexpr gpio_num_t IR_RIGHT = GPIO_NUM_34;

    constexpr gpio_num_t I2C_SDA = GPIO_NUM_21;
    constexpr gpio_num_t I2C_SCL = GPIO_NUM_22;

    constexpr gpio_num_t LED_STRIP_DATA = GPIO_NUM_4;

    constexpr float LED_LUX_ON_THRESHOLD = 25.0f;  // LED ON if darker than this
    constexpr float LED_LUX_OFF_THRESHOLD = 32.0f; // LED OFF if brighter than this

    constexpr bool IR_ACTIVE_LOW = true;

    constexpr gpio_num_t I2S_BCLK = GPIO_NUM_27;
    constexpr gpio_num_t I2S_LRCLK = GPIO_NUM_14;
    constexpr gpio_num_t I2S_DOUT = GPIO_NUM_12;
}
