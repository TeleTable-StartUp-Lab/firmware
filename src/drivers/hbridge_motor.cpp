#include "drivers/hbridge_motor.h"
#include <cmath>

namespace
{
    uint8_t nextLedcChannel()
    {
        static uint8_t ch = 0;
        return ch++;
    }
}

HBridgeMotor::HBridgeMotor(const Config &cfg)
    : cfg_(cfg), ch1_(nextLedcChannel()), ch2_(nextLedcChannel()) {}

void HBridgeMotor::begin()
{
    ledcSetup(ch1_, cfg_.pwm_hz, cfg_.pwm_resolution_bits);
    ledcSetup(ch2_, cfg_.pwm_hz, cfg_.pwm_resolution_bits);

    ledcAttachPin(static_cast<uint8_t>(cfg_.in1), ch1_);
    ledcAttachPin(static_cast<uint8_t>(cfg_.in2), ch2_);

    stop();
}

void HBridgeMotor::set(float command)
{
    command = clamp_(command, -1.0f, 1.0f);

    const uint32_t maxDuty = (1u << cfg_.pwm_resolution_bits) - 1u;
    const uint32_t duty =
        static_cast<uint32_t>(std::lround(std::fabs(command) * maxDuty));

    if (command > 0.001f)
    {
        writeDuty_(ch1_, duty);
        writeDuty_(ch2_, 0);
    }
    else if (command < -0.001f)
    {
        writeDuty_(ch1_, 0);
        writeDuty_(ch2_, duty);
    }
    else
    {
        stop();
    }
}

void HBridgeMotor::stop()
{
    writeDuty_(ch1_, 0);
    writeDuty_(ch2_, 0);
}

void HBridgeMotor::writeDuty_(uint8_t ch, uint32_t duty)
{
    ledcWrite(ch, duty);
}

float HBridgeMotor::clamp_(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}
