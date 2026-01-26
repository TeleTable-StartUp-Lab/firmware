#include "audio.h"
#include "pins.h"
#include <math.h>

static float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

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
    _started = true;
    return true;
}

void Audio::end()
{
    if (!_started)
        return;
    stop();
    i2s_driver_uninstall(_port);
    _started = false;
}

void Audio::setVolume(float v)
{
    _volume = clampf(v, 0.0f, 1.0f);
}

float Audio::volume() const
{
    return _volume;
}

void Audio::stop()
{
    if (!_started)
        return;
    i2s_zero_dma_buffer(_port);
}

void Audio::writeSamplesMono16(const int16_t *samples, size_t count)
{
    size_t bytesWritten = 0;
    i2s_write(_port, samples, count * sizeof(int16_t), &bytesWritten, portMAX_DELAY);
}

void Audio::playSine(float freqHz, uint32_t durationMs)
{
    if (!_started)
        return;

    static constexpr int N = 256;
    int16_t buf[N];

    uint32_t start = millis();
    while (millis() - start < durationMs)
    {
        for (int i = 0; i < N; i++)
        {
            float t = (float)(_sampleIndex++) / (float)AUDIO_SAMPLE_RATE;
            float s = sinf(2.0f * PI * freqHz * t);
            float a = s * 32767.0f * _volume;
            if (a > 32767.0f)
                a = 32767.0f;
            if (a < -32768.0f)
                a = -32768.0f;
            buf[i] = (int16_t)a;
        }
        writeSamplesMono16(buf, N);
    }
}

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

struct Note
{
    const char *name;
    int ms;
};

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

void Audio::playMelodyLoop()
{
    if (!_started)
        return;

    for (size_t i = 0; i < sizeof(melody) / sizeof(melody[0]); i++)
    {
        float f = noteFreq(melody[i].name);
        playSine(f, (uint32_t)melody[i].ms);
    }
}

bool Audio::playWavFromFS(const char *path)
{
    (void)path;
    return false;
}
