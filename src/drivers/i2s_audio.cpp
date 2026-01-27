#include "drivers/i2s_audio.h"
#include <cmath>
#include <driver/i2s.h>

I2sAudio::I2sAudio(const Config &cfg) : cfg_(cfg) {}

bool I2sAudio::begin()
{
    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = (int)cfg_.sample_rate_hz,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // mono
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0};

    const i2s_pin_config_t pin_config = {
        .bck_io_num = cfg_.bclk_pin,
        .ws_io_num = cfg_.lrclk_pin,
        .data_out_num = cfg_.dout_pin,
        .data_in_num = I2S_PIN_NO_CHANGE};

    if (i2s_driver_install(I2S_NUM_0, &i2s_config, 0, nullptr) != ESP_OK)
        return false;
    if (i2s_set_pin(I2S_NUM_0, &pin_config) != ESP_OK)
        return false;
    if (i2s_zero_dma_buffer(I2S_NUM_0) != ESP_OK)
        return false;

    ok_ = true;
    return true;
}

void I2sAudio::setVolume(float v)
{
    if (v < 0.0f)
        v = 0.0f;
    if (v > 1.0f)
        v = 1.0f;
    volume_ = v;
}

float I2sAudio::volume() const { return volume_; }

void I2sAudio::playBeep(uint16_t freq_hz, uint16_t duration_ms)
{
    if (!ok_)
        return;

    const uint32_t sr = cfg_.sample_rate_hz;
    const uint32_t total_samples = (uint32_t)((uint64_t)sr * duration_ms / 1000ULL);

    const float amp = 0.25f * volume_; // safe headroom
    const float w = 2.0f * 3.14159265358979323846f * (float)freq_hz / (float)sr;

    // Small chunk buffer
    int16_t buf[256];
    uint32_t produced = 0;

    while (produced < total_samples)
    {
        const uint32_t chunk = std::min<uint32_t>(256, total_samples - produced);

        for (uint32_t i = 0; i < chunk; ++i)
        {
            const float s = sinf(w * (float)(produced + i));
            const int16_t sample = (int16_t)(s * 32767.0f * amp);
            buf[i] = sample;
        }

        size_t bytes_written = 0;
        i2s_write(I2S_NUM_0, buf, chunk * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        produced += chunk;
    }
}
