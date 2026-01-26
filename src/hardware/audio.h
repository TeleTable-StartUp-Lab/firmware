#pragma once
#include <Arduino.h>
#include <driver/i2s.h>

class Audio
{
public:
    bool begin();
    void end();

    void setVolume(float v);
    float volume() const;

    void stop();

    void playSine(float freqHz, uint32_t durationMs);
    void playMelodyLoop();

    bool playWavFromFS(const char *path);

private:
    void writeSamplesMono16(const int16_t *samples, size_t count);

    bool _started = false;
    float _volume = 0.4f;

    i2s_port_t _port = I2S_NUM_0;
    int32_t _sampleIndex = 0;
};
