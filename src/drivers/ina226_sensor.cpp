#include "drivers/ina226_sensor.h"

#include <cmath>

namespace
{
    constexpr uint8_t REG_CONFIG = 0x00;
    constexpr uint8_t REG_SHUNT_VOLTAGE = 0x01;
    constexpr uint8_t REG_BUS_VOLTAGE = 0x02;

    constexpr uint16_t CONFIG_CONTINUOUS_SHUNT_AND_BUS = 0x0927;

    constexpr float SHUNT_VOLTAGE_LSB_V = 0.0000025f;
    constexpr float BUS_VOLTAGE_LSB_V = 0.00125f;
}

Ina226Sensor::Ina226Sensor(const Config &cfg) : cfg_(cfg) {}

bool Ina226Sensor::begin()
{
    if (!cfg_.wire || cfg_.shunt_ohms <= 0.0f || cfg_.battery_full_voltage <= cfg_.battery_empty_voltage)
        return false;

    cfg_.wire->beginTransmission(cfg_.address);
    if (cfg_.wire->endTransmission() != 0)
    {
        ok_ = false;
        return false;
    }

    ok_ = writeRegister_(REG_CONFIG, CONFIG_CONTINUOUS_SHUNT_AND_BUS);
    if (!ok_)
        return false;

    delay(5);
    has_reading_ = readSample_();
    last_sample_ms_ = millis();
    return ok_;
}

void Ina226Sensor::update(uint32_t now_ms)
{
    if (!ok_)
        return;
    if (now_ms - last_sample_ms_ < cfg_.sample_period_ms)
        return;

    last_sample_ms_ = now_ms;
    if (readSample_())
        has_reading_ = true;
}

bool Ina226Sensor::isOk() const
{
    return ok_;
}

bool Ina226Sensor::hasReading() const
{
    return has_reading_;
}

const Ina226Sensor::Reading &Ina226Sensor::reading() const
{
    return reading_;
}

bool Ina226Sensor::writeRegister_(uint8_t reg, uint16_t value)
{
    cfg_.wire->beginTransmission(cfg_.address);
    cfg_.wire->write(reg);
    cfg_.wire->write(static_cast<uint8_t>((value >> 8) & 0xFF));
    cfg_.wire->write(static_cast<uint8_t>(value & 0xFF));
    return cfg_.wire->endTransmission() == 0;
}

bool Ina226Sensor::readRegister_(uint8_t reg, uint16_t &value)
{
    cfg_.wire->beginTransmission(cfg_.address);
    cfg_.wire->write(reg);
    if (cfg_.wire->endTransmission(false) != 0)
        return false;

    const uint8_t requested = cfg_.wire->requestFrom(static_cast<int>(cfg_.address), 2);
    if (requested != 2)
        return false;

    value = static_cast<uint16_t>(cfg_.wire->read()) << 8;
    value |= static_cast<uint16_t>(cfg_.wire->read());
    return true;
}

bool Ina226Sensor::readSample_()
{
    uint16_t shunt_raw = 0;
    uint16_t bus_raw = 0;

    if (!readRegister_(REG_SHUNT_VOLTAGE, shunt_raw) || !readRegister_(REG_BUS_VOLTAGE, bus_raw))
        return false;

    const int16_t signed_shunt_raw = static_cast<int16_t>(shunt_raw);
    const float shunt_voltage_v = static_cast<float>(signed_shunt_raw) * SHUNT_VOLTAGE_LSB_V;
    const float bus_voltage_v = static_cast<float>(bus_raw) * BUS_VOLTAGE_LSB_V;
    const float current_a = shunt_voltage_v / cfg_.shunt_ohms;

    reading_.shunt_voltage_mv = shunt_voltage_v * 1000.0f;
    reading_.bus_voltage_v = bus_voltage_v;
    reading_.current_a = current_a;
    reading_.power_w = bus_voltage_v * current_a;
    reading_.battery_percent = mapBatteryPercent_(bus_voltage_v);
    return true;
}

int Ina226Sensor::mapBatteryPercent_(float bus_voltage_v) const
{
    const float span = cfg_.battery_full_voltage - cfg_.battery_empty_voltage;
    if (span <= 0.0f)
        return 0;

    const float ratio = (bus_voltage_v - cfg_.battery_empty_voltage) / span;
    const float percent = ratio * 100.0f;
    return static_cast<int>(lroundf(constrain(percent, 0.0f, 100.0f)));
}
