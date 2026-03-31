#pragma once

#include <Arduino.h>
#include <Wire.h>

class Ina226Sensor
{
public:
    struct Config
    {
        TwoWire *wire;
        uint8_t address;
        uint32_t sample_period_ms;
        float shunt_ohms;
        float battery_empty_voltage;
        float battery_full_voltage;
    };

    struct Reading
    {
        float shunt_voltage_mv = 0.0f;
        float bus_voltage_v = 0.0f;
        float current_a = 0.0f;
        float power_w = 0.0f;
        int battery_percent = 0;
    };

    explicit Ina226Sensor(const Config &cfg);

    bool begin();
    void update(uint32_t now_ms);

    bool isOk() const;
    bool hasReading() const;
    const Reading &reading() const;

private:
    Config cfg_;
    bool ok_ = false;
    bool has_reading_ = false;
    uint32_t last_sample_ms_ = 0;
    Reading reading_;

    bool writeRegister_(uint8_t reg, uint16_t value);
    bool readRegister_(uint8_t reg, uint16_t &value);
    bool readSample_();
    int mapBatteryPercent_(float bus_voltage_v) const;
};
