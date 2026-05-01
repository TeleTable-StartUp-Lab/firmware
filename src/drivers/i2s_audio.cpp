#include "drivers/i2s_audio.h"

#include <algorithm>
#include <cmath>
#include <driver/i2s.h>

I2sAudio::I2sAudio(const Config &cfg) : cfg_(cfg) {}

namespace
{
#if defined(I2S_COMM_FORMAT_STAND_I2S)
    constexpr i2s_comm_format_t kI2sCommFormat = I2S_COMM_FORMAT_STAND_I2S;
#else
    constexpr i2s_comm_format_t kI2sCommFormat = I2S_COMM_FORMAT_I2S;
#endif
}

bool I2sAudio::begin()
{
    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = (int)cfg_.sample_rate_hz,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = kI2sCommFormat,
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
    if (total_samples == 0)
        return;

    const float amp = 0.25f * volume_; // safe headroom
    const float w = 2.0f * 3.14159265358979323846f * (float)freq_hz / (float)sr;
    float phase = 0.0f;

    // Duplicate mono samples onto both I2S slots.
    int16_t buf[512];
    uint32_t produced = 0;

    while (produced < total_samples)
    {
        const uint32_t chunk = std::min<uint32_t>(256, total_samples - produced);

        for (uint32_t i = 0; i < chunk; ++i)
        {
            phase += w;
            const float s = sinf(phase);
            const int16_t sample = (int16_t)(s * 32767.0f * amp);
            buf[(i * 2) + 0] = sample;
            buf[(i * 2) + 1] = sample;
        }

        size_t bytes_written = 0;
        i2s_write(I2S_NUM_0, buf, chunk * 2 * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        produced += chunk;
    }
}

bool I2sAudio::startStream(uint32_t sample_rate_hz, uint8_t channels, uint8_t bits_per_sample, bool little_endian)
{
    if (!ok_)
        return false;
    if (channels != 1 || bits_per_sample != 16 || !little_endian)
        return false;

    if (i2s_set_clk(I2S_NUM_0, static_cast<uint32_t>(sample_rate_hz), I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO) != ESP_OK)
        return false;

    stream_sample_rate_hz_ = sample_rate_hz;
    stream_channels_ = channels;
    stream_bits_per_sample_ = bits_per_sample;
    stream_little_endian_ = little_endian;

    pcm_write_idx_ = 0;
    pcm_read_idx_ = 0;
    pcm_available_ = 0;
    i2s_zero_dma_buffer(I2S_NUM_0);

    stream_active_ = true;
    return true;
}

void I2sAudio::stopStream()
{
    if (!ok_)
        return;

    stream_active_ = false;
    pcm_write_idx_ = 0;
    pcm_read_idx_ = 0;
    pcm_available_ = 0;
    i2s_zero_dma_buffer(I2S_NUM_0);
}

bool I2sAudio::enqueuePcmBytes(const uint8_t *data, size_t len)
{
    if (!ok_ || !stream_active_)
        return false;
    if (stream_channels_ != 1 || stream_bits_per_sample_ != 16 || !stream_little_endian_)
        return false;
    if (!data || len < 2)
        return false;

    const size_t total_samples = len / 2;
    size_t writable = kPcmBufferSamples - pcm_available_;
    if (!writable)
        return false;

    const size_t to_write = std::min(writable, total_samples);
    for (size_t i = 0; i < to_write; ++i)
    {
        const size_t byte_idx = i * 2;
        const uint16_t lo = data[byte_idx];
        const uint16_t hi = data[byte_idx + 1];
        const int16_t sample = static_cast<int16_t>((hi << 8) | lo);

        pcm_buffer_[pcm_write_idx_] = sample;
        pcm_write_idx_ = (pcm_write_idx_ + 1) % kPcmBufferSamples;
    }
    pcm_available_ += to_write;
    return to_write == total_samples;
}

void I2sAudio::loop()
{
    if (!ok_ || !stream_active_ || pcm_available_ == 0)
        return;

    constexpr size_t kChunkSamples = 256;
    const size_t chunk = std::min(kChunkSamples, pcm_available_);
    int16_t out[kChunkSamples * 2];

    for (size_t i = 0; i < chunk; ++i)
    {
        const int16_t sample = pcm_buffer_[pcm_read_idx_];
        pcm_read_idx_ = (pcm_read_idx_ + 1) % kPcmBufferSamples;

        float scaled = static_cast<float>(sample) * volume_;
        if (scaled > 32767.0f)
            scaled = 32767.0f;
        if (scaled < -32768.0f)
            scaled = -32768.0f;
        const int16_t out_sample = static_cast<int16_t>(scaled);

        out[(i * 2) + 0] = out_sample;
        out[(i * 2) + 1] = out_sample;
    }

    pcm_available_ -= chunk;

    size_t bytes_written = 0;
    i2s_write(I2S_NUM_0, out, chunk * 2 * sizeof(int16_t), &bytes_written, 10);
}

bool I2sAudio::isStreaming() const
{
    return stream_active_;
}
