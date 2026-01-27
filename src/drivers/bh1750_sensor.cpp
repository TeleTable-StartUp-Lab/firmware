#include "drivers/bh1750_sensor.h"

namespace
{
    constexpr uint8_t CMD_POWER_ON = 0x01;
    constexpr uint8_t CMD_RESET = 0x07;
    constexpr uint8_t CMD_CONT_HIRES = 0x10; // Continuous H-Resolution Mode
}

Bh1750Sensor::Bh1750Sensor(const Config &cfg) : cfg_(cfg) {}

bool Bh1750Sensor::begin()
{
    if (!cfg_.wire)
        return false;

    cfg_.wire->beginTransmission(cfg_.address);
    if (cfg_.wire->endTransmission() != 0)
    {
        ok_ = false;
        return false;
    }

    cfg_.wire->beginTransmission(cfg_.address);
    cfg_.wire->write(CMD_POWER_ON);
    if (cfg_.wire->endTransmission() != 0)
    {
        ok_ = false;
        return false;
    }

    cfg_.wire->beginTransmission(cfg_.address);
    cfg_.wire->write(CMD_RESET);
    cfg_.wire->endTransmission();

    ok_ = startContinuousHighRes_();
    last_sample_ms_ = millis();
    return ok_;
}

void Bh1750Sensor::update(uint32_t now_ms)
{
    if (!ok_)
        return;
    if (now_ms - last_sample_ms_ < cfg_.sample_period_ms)
        return;
    last_sample_ms_ = now_ms;

    has_reading_ = readLux_();
}

bool Bh1750Sensor::hasReading() const { return has_reading_; }
float Bh1750Sensor::lux() const { return lux_; }

bool Bh1750Sensor::startContinuousHighRes_()
{
    cfg_.wire->beginTransmission(cfg_.address);
    cfg_.wire->write(CMD_CONT_HIRES);
    return cfg_.wire->endTransmission() == 0;
}

bool Bh1750Sensor::readLux_()
{
    const uint8_t n = cfg_.wire->requestFrom(static_cast<int>(cfg_.address), 2);
    if (n != 2)
        return false;

    const uint16_t raw = (static_cast<uint16_t>(cfg_.wire->read()) << 8) |
                         static_cast<uint16_t>(cfg_.wire->read());

    // Typical conversion factor: lux = raw / 1.2 in high-res mode
    lux_ = static_cast<float>(raw) / 1.2f;
    return true;
}
