#include "drivers/obstacle_sensor.h"

ObstacleSensor::ObstacleSensor(const Config &cfg) : cfg_(cfg) {}

void ObstacleSensor::begin()
{
    const uint8_t pin = static_cast<uint8_t>(cfg_.pin);

    if (cfg_.use_internal_pullup)
    {
        pinMode(pin, INPUT_PULLUP);
    }
    else
    {
        pinMode(pin, INPUT);
    }

    const bool raw = readRaw_();
    stable_ = rawToObstacle_(raw);
    last_stable_ = stable_;
    pending_ = stable_;
    last_change_ms_ = millis();
}

void ObstacleSensor::update(uint32_t now_ms)
{
    rose_ = false;
    fell_ = false;

    const bool raw = readRaw_();
    const bool obs = rawToObstacle_(raw);

    if (obs != pending_)
    {
        pending_ = obs;
        last_change_ms_ = now_ms;
    }

    if (pending_ != stable_)
    {
        if (now_ms - last_change_ms_ >= cfg_.debounce_ms)
        {
            last_stable_ = stable_;
            stable_ = pending_;

            if (!last_stable_ && stable_)
                rose_ = true;
            if (last_stable_ && !stable_)
                fell_ = true;
        }
    }
}

bool ObstacleSensor::isObstacle() const
{
    return stable_;
}

bool ObstacleSensor::roseObstacle()
{
    const bool v = rose_;
    rose_ = false;
    return v;
}

bool ObstacleSensor::fellObstacle()
{
    const bool v = fell_;
    fell_ = false;
    return v;
}

bool ObstacleSensor::readRaw_() const
{
    return digitalRead(static_cast<uint8_t>(cfg_.pin)) != 0;
}

bool ObstacleSensor::rawToObstacle_(bool raw) const
{
    // raw=true means HIGH at pin
    // If active_low: obstacle when LOW => obstacle = !raw
    return cfg_.active_low ? !raw : raw;
}
