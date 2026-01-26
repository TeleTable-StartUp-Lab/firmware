#pragma once
#include <Arduino.h>
#include "pins.h"
#include "core/system_state.h"

class Motor
{
public:
    Motor(uint8_t in1, uint8_t in2, uint8_t ch1, uint8_t ch2);
    void begin(uint32_t freq, uint8_t resBits);
    void stop();
    void set(int16_t speed);

private:
    uint8_t _in1, _in2, _ch1, _ch2;
};

class DifferentialDrive
{
public:
    void begin();
    void stop();
    void drive(int16_t left, int16_t right);
    void twist(float linear, float angular);

private:
    Motor _left{L_MOTOR_IN1, L_MOTOR_IN2, CH_L1, CH_L2};
    Motor _right{R_MOTOR_IN1, R_MOTOR_IN2, CH_R1, CH_R2};

    static int16_t clampI16(int32_t v, int16_t lo, int16_t hi);
    static float clampF(float v, float lo, float hi);
};

class MotorController
{
public:
    explicit MotorController(SystemState *state);
    void begin();
    void startTask();
    void stopTask();

private:
    static void taskTrampoline(void *arg);
    void taskLoop();

    SystemState *_state;
    DifferentialDrive _drive;

    TaskHandle_t _task = nullptr;
};
