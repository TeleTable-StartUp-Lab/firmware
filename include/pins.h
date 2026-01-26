#pragma once

// IR Sensors
constexpr int IR_LEFT_PIN = 18;
constexpr int IR_RIGHT_PIN = 19;

// I2C
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

// LED Strip
constexpr int LED_DATA_PIN = 13;
constexpr int NUM_LEDS = 144;

// Motors (H-bridge IN pins)
#define L_MOTOR_IN1 25
#define L_MOTOR_IN2 26
#define R_MOTOR_IN1 27
#define R_MOTOR_IN2 14

// PWM channels
#define CH_L1 0
#define CH_L2 1
#define CH_R1 2
#define CH_R2 3

// Audio (I2S) - avoid motor pins
constexpr int I2S_DOUT = 23;
constexpr int I2S_BCLK = 4;
constexpr int I2S_LRC = 16;

constexpr int AUDIO_SAMPLE_RATE = 44100;
