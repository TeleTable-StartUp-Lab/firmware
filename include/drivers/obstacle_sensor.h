#pragma once
#include <Arduino.h>

class ObstacleSensor
{
public:
    struct Config
    {
        gpio_num_t pin;
        bool active_low;
        uint32_t debounce_ms;
        bool use_internal_pullup;
    };

    explicit ObstacleSensor(const Config &cfg);

    void begin();
    void update(uint32_t now_ms);

    bool isObstacle() const;
    bool roseObstacle(); // true once when it becomes obstacle
    bool fellObstacle(); // true once when it becomes clear

private:
    Config cfg_;

    bool stable_ = false;
    bool last_stable_ = false;

    bool pending_ = false;
    uint32_t last_change_ms_ = 0;

    bool rose_ = false;
    bool fell_ = false;

    bool readRaw_() const;
    bool rawToObstacle_(bool raw) const;
};
