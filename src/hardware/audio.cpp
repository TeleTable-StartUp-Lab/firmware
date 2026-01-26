#include "hardware/audio.h"
#include "pins.h"
#include <math.h>
#include <string.h>

static float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

struct Note
{
    const char *name;
    uint16_t ms;
};

static float noteFreq(const char *n)
{
    if (!strcmp(n, "C4"))
        return 261.63f;
    if (!strcmp(n, "D4"))
        return 293.66f;
    if (!strcmp(n, "E4"))
        return 329.63f;
    if (!strcmp(n, "F4"))
        return 349.23f;
    if (!strcmp(n, "G4"))
        return 392.00f;
    if (!strcmp(n, "A4"))
        return 440.00f;
    if (!strcmp(n, "B4"))
        return 493.88f;
    if (!strcmp(n, "C5"))
        return 523.25f;
    return 440.0f;
}

static const Note melody[] = {
    {"E4", 280},
    {"E4", 280},
    {"F4", 280},
    {"G4", 280},
    {"G4", 280},
    {"F4", 280},
    {"E4", 280},
    {"D4", 280},
    {"C4", 280},
    {"C4", 280},
    {"D4", 280},
    {"E4", 280},
    {"E4", 420},
    {"D4", 140},
    {"D4", 560},

    {"E4", 280},
    {"E4", 280},
    {"F4", 280},
    {"G4", 280},
    {"G4", 280},
    {"F4", 280},
    {"E4", 280},
    {"D4", 280},
    {"C4", 280},
    {"C4", 280},
    {"D4", 280},
    {"E4", 280},
    {"D4", 420},
    {"C4", 140},
    {"C4", 560},
};

bool Audio::begin()
{
    if (_started)
        return true;

    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = AUDIO_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0};

    i2s_pin_config_t pins = {
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRC,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE};

    if (i2s_driver_install(_port, &cfg, 0, nullptr) != ESP_OK)
        return false;
    if (i2s_set_pin(_port, &pins) != ESP_OK)
        return false;

    i2s_zero_dma_buffer(_port);

    _q = xQueueCreate(8, sizeof(AudioCommand));
    if (!_q)
        return false;

    _started = true;
    return true;
}

void Audio::end()
{
    if (!_started)
        return;

    stopTask();
    i2s_zero_dma_buffer(_port);

    if (_q)
    {
        vQueueDelete(_q);
        _q = nullptr;
    }

    i2s_driver_uninstall(_port);
    _started = false;
}

void Audio::startTask()
{
    if (!_started || _task)
        return;

    xTaskCreatePinnedToCore(
        taskTrampoline,
        "AudioTask",
        4096,
        this,
        3,
        &_task,
        1);
}

void Audio::stopTask()
{
    if (!_task)
        return;

    vTaskDelete(_task);
    _task = nullptr;
}

void Audio::cmdStop()
{
    if (!_q)
        return;

    AudioCommand c{AudioCmdType::Stop, 0.0f, 0.0f};
    xQueueSend(_q, &c, 0);
}

void Audio::cmdSetVolume(float v)
{
    if (!_q)
        return;

    AudioCommand c{AudioCmdType::SetVolume, v, 0.0f};
    xQueueSend(_q, &c, 0);
}

void Audio::cmdPlayMelody(bool loop)
{
    if (!_q)
        return;

    AudioCommand c{AudioCmdType::PlayMelody, loop ? 1.0f : 0.0f, 0.0f};
    xQueueSend(_q, &c, 0);
}

void Audio::cmdPlayTone(float freqHz, uint32_t ms)
{
    if (!_q)
        return;

    AudioCommand c{AudioCmdType::PlayTone, freqHz, (float)ms};
    xQueueSend(_q, &c, 0);
}

void Audio::taskTrampoline(void *arg)
{
    static_cast<Audio *>(arg)->taskLoop();
}

void Audio::i2sWriteMono16(const int16_t *samples, size_t n)
{
    size_t bytes = 0;
    i2s_write(_port, samples, n * sizeof(int16_t), &bytes, portMAX_DELAY);
}

void Audio::renderSineChunk(float freqHz, int16_t *buf, size_t n)
{
    const float vol = _volume;

    for (size_t i = 0; i < n; i++)
    {
        float t = (float)(_sampleIndex++) / (float)AUDIO_SAMPLE_RATE;
        float s = sinf(2.0f * PI * freqHz * t);
        float a = s * 32767.0f * vol;

        if (a > 32767.0f)
            a = 32767.0f;
        if (a < -32768.0f)
            a = -32768.0f;

        buf[i] = (int16_t)a;
    }
}

void Audio::taskLoop()
{
    static constexpr size_t N = 256;
    int16_t buf[N];

    bool playing = false;
    bool loopMel = true;

    enum Mode
    {
        Idle,
        Melody,
        Tone
    } mode = Idle;

    size_t melIdx = 0;
    uint32_t noteLeftMs = 0;
    float curFreq = 440.0f;

    uint32_t toneLeftMs = 0;

    for (;;)
    {
        AudioCommand cmd;
        while (_q && xQueueReceive(_q, &cmd, 0) == pdTRUE)
        {
            if (cmd.type == AudioCmdType::Stop)
            {
                playing = false;
                mode = Idle;
                i2s_zero_dma_buffer(_port);
            }
            else if (cmd.type == AudioCmdType::SetVolume)
            {
                _volume = clampf(cmd.a, 0.0f, 1.0f);
            }
            else if (cmd.type == AudioCmdType::PlayMelody)
            {
                loopMel = (cmd.a > 0.5f);
                playing = true;
                mode = Melody;

                melIdx = 0;
                noteLeftMs = melody[0].ms;
                curFreq = noteFreq(melody[0].name);
            }
            else if (cmd.type == AudioCmdType::PlayTone)
            {
                playing = true;
                mode = Tone;

                curFreq = cmd.a;
                toneLeftMs = (uint32_t)cmd.b;
            }
        }

        if (!playing || mode == Idle)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        const uint32_t chunkMs = (uint32_t)((1000.0f * (float)N) / (float)AUDIO_SAMPLE_RATE);

        renderSineChunk(curFreq, buf, N);
        i2sWriteMono16(buf, N);

        if (mode == Tone)
        {
            if (toneLeftMs <= chunkMs)
            {
                playing = false;
                mode = Idle;
            }
            else
            {
                toneLeftMs -= chunkMs;
            }
        }
        else if (mode == Melody)
        {
            if (noteLeftMs <= chunkMs)
            {
                melIdx++;
                if (melIdx >= (sizeof(melody) / sizeof(melody[0])))
                {
                    if (!loopMel)
                    {
                        playing = false;
                        mode = Idle;
                        continue;
                    }
                    melIdx = 0;
                }

                noteLeftMs = melody[melIdx].ms;
                curFreq = noteFreq(melody[melIdx].name);
            }
            else
            {
                noteLeftMs -= chunkMs;
            }
        }
    }
}
