#pragma once
#include <Arduino.h>

class I2sAudio
{
public:
    struct Config
    {
        int bclk_pin;
        int lrclk_pin;
        int dout_pin;
        uint32_t sample_rate_hz;
    };

    explicit I2sAudio(const Config &cfg);

    bool begin();
    void setVolume(float v); // 0..1
    float volume() const;

    void playBeep(uint16_t freq_hz, uint16_t duration_ms);

private:
    Config cfg_;
    float volume_ = 0.2f;
    bool ok_ = false;
};
