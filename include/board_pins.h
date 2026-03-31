#pragma once
#include <Arduino.h>

namespace BoardPins
{
    constexpr uint8_t OLED_I2C_ADDRESS = 0x3C;
    constexpr uint8_t BH1750_I2C_ADDRESS = 0x23;
    constexpr uint8_t INA226_I2C_ADDRESS = 0x40;
    constexpr uint8_t MPU6050_I2C_ADDRESS = 0x68;
    constexpr uint8_t MPU6050_I2C_ADDRESS_ALT = 0x69;

    constexpr int8_t OLED_RESET_PIN = -1;

    constexpr gpio_num_t LEFT_MOTOR_IN1 = GPIO_NUM_25;
    constexpr gpio_num_t LEFT_MOTOR_IN2 = GPIO_NUM_26;

    constexpr gpio_num_t RIGHT_MOTOR_IN1 = GPIO_NUM_15;
    constexpr gpio_num_t RIGHT_MOTOR_IN2 = GPIO_NUM_2;

    constexpr gpio_num_t IR_LEFT = GPIO_NUM_32;
    constexpr gpio_num_t IR_MIDDLE = GPIO_NUM_33;
    constexpr gpio_num_t IR_RIGHT = GPIO_NUM_34;

    constexpr gpio_num_t I2C_SDA = GPIO_NUM_21;
    constexpr gpio_num_t I2C_SCL = GPIO_NUM_22;

    // Defaults are for a 1-cell Li-Ion pack. Adjust them to your INA226 shunt and battery pack.
    constexpr float INA226_SHUNT_OHMS = 0.01f;
    constexpr float BATTERY_EMPTY_VOLTAGE = 3.20f;
    constexpr float BATTERY_FULL_VOLTAGE = 4.20f;

    constexpr gpio_num_t RC522_SDA_SS = GPIO_NUM_5;
    constexpr gpio_num_t RC522_SCK = GPIO_NUM_18;
    constexpr gpio_num_t RC522_MOSI = GPIO_NUM_23;
    constexpr gpio_num_t RC522_MISO = GPIO_NUM_19;
    constexpr gpio_num_t RC522_RST = GPIO_NUM_13;

    constexpr gpio_num_t LED_STRIP_DATA = GPIO_NUM_4;

    constexpr float LED_LUX_ON_THRESHOLD = 25.0f;  // LED ON if darker than this
    constexpr float LED_LUX_OFF_THRESHOLD = 32.0f; // LED OFF if brighter than this

    constexpr bool IR_ACTIVE_LOW = true;

    constexpr gpio_num_t I2S_BCLK = GPIO_NUM_27;
    constexpr gpio_num_t I2S_LRCLK = GPIO_NUM_14;
    constexpr gpio_num_t I2S_DOUT = GPIO_NUM_16;
}
