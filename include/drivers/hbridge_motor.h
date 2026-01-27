#pragma once
#include <Arduino.h>

class HBridgeMotor
{
public:
    struct Config
    {
        gpio_num_t in1;
        gpio_num_t in2;
        uint32_t pwm_hz;
        uint8_t pwm_resolution_bits;
    };

    explicit HBridgeMotor(const Config &cfg);

    void begin();

    // Command in range [-1.0 .. +1.0]
    void set(float command);

    // Coast stop: both outputs low
    void stop();

private:
    Config cfg_;
    uint8_t ch1_;
    uint8_t ch2_;

    void writeDuty_(uint8_t ch, uint32_t duty);
    static float clamp_(float v, float lo, float hi);
};
