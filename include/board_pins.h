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

    constexpr gpio_num_t LEFT_MOTOR_IN1 = GPIO_NUM_25; // board label: IO25
    constexpr gpio_num_t LEFT_MOTOR_IN2 = GPIO_NUM_26; // board label: IO26

    constexpr gpio_num_t RIGHT_MOTOR_IN1 = GPIO_NUM_32; // board label: IO32
    constexpr gpio_num_t RIGHT_MOTOR_IN2 = GPIO_NUM_33; // board label: IO33

    constexpr gpio_num_t IR_LEFT = GPIO_NUM_34;   // board label: IO34, input only
    constexpr gpio_num_t IR_MIDDLE = GPIO_NUM_35; // board label: IO35, input only
    constexpr gpio_num_t IR_RIGHT = GPIO_NUM_39;  // board label: SVN, input only

    constexpr gpio_num_t I2C_SDA = GPIO_NUM_21; // board label: IO21
    constexpr gpio_num_t I2C_SCL = GPIO_NUM_22; // board label: IO22

    // Defaults are for a 1-cell Li-Ion pack. Adjust them to your INA226 shunt and battery pack.
    constexpr float INA226_SHUNT_OHMS = 0.01f;
    constexpr float BATTERY_EMPTY_VOLTAGE = 9.6f;
    constexpr float BATTERY_FULL_VOLTAGE = 12.6;

    constexpr gpio_num_t RC522_SDA_SS = GPIO_NUM_13; // board label: IO13
    constexpr gpio_num_t RC522_SCK = GPIO_NUM_18;    // board label: IO18
    constexpr gpio_num_t RC522_MOSI = GPIO_NUM_23;   // board label: IO23
    constexpr gpio_num_t RC522_MISO = GPIO_NUM_19;   // board label: IO19
    constexpr gpio_num_t RC522_RST = GPIO_NUM_27;    // board label: IO27

    constexpr gpio_num_t LED_STRIP_DATA = GPIO_NUM_4; // board label: IO4

    constexpr float LED_LUX_ON_THRESHOLD = 25.0f;  // LED ON if darker than this
    constexpr float LED_LUX_OFF_THRESHOLD = 32.0f; // LED OFF if brighter than this

    constexpr bool IR_ACTIVE_LOW = true;

    constexpr gpio_num_t I2S_BCLK = GPIO_NUM_14;  // board label: IO14
    constexpr gpio_num_t I2S_LRCLK = GPIO_NUM_16; // board label: IO16
    constexpr gpio_num_t I2S_DOUT = GPIO_NUM_17;  // board label: IO17
}
