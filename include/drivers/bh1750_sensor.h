#pragma once
#include <Arduino.h>
#include <Wire.h>

class Bh1750Sensor
{
public:
    enum class Address : uint8_t
    {
        A0 = 0x23,
        A1 = 0x5C
    };

    struct Config
    {
        TwoWire *wire;
        uint8_t address;
        uint32_t sample_period_ms;
    };

    explicit Bh1750Sensor(const Config &cfg);

    bool begin();
    void update(uint32_t now_ms);

    bool hasReading() const;
    float lux() const;

private:
    Config cfg_;
    bool ok_ = false;

    uint32_t last_sample_ms_ = 0;
    bool has_reading_ = false;
    float lux_ = 0.0f;

    bool startContinuousHighRes_();
    bool readLux_();
};
