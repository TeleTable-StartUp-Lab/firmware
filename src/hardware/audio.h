#pragma once
#include <Arduino.h>
#include <driver/i2s.h>

enum class AudioCmdType : uint8_t
{
    Stop,
    SetVolume,
    PlayMelody,
    PlayTone
};

struct AudioCommand
{
    AudioCmdType type;
    float a;
    float b;
};

class Audio
{
public:
    bool begin();
    void end();

    void startTask();
    void stopTask();

    bool isStarted() const { return _started; }
    bool isRunning() const { return _task != nullptr; }

    void cmdStop();
    void cmdSetVolume(float v);
    void cmdPlayMelody(bool loop = true);
    void cmdPlayTone(float freqHz, uint32_t ms);

private:
    static void taskTrampoline(void *arg);
    void taskLoop();

    void i2sWriteMono16(const int16_t *samples, size_t n);
    void renderSineChunk(float freqHz, int16_t *buf, size_t n);

    bool _started = false;
    float _volume = 0.4f;

    i2s_port_t _port = I2S_NUM_0;
    int32_t _sampleIndex = 0;

    TaskHandle_t _task = nullptr;
    QueueHandle_t _q = nullptr;
};
