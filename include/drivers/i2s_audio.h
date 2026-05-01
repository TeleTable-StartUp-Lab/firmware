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

    bool startStream(uint32_t sample_rate_hz, uint8_t channels, uint8_t bits_per_sample, bool little_endian);
    void stopStream();
    bool enqueuePcmBytes(const uint8_t *data, size_t len);
    void loop();
    bool isStreaming() const;

private:
    Config cfg_;
    float volume_ = 0.2f;
    bool ok_ = false;
    bool stream_active_ = false;
    uint32_t stream_sample_rate_hz_ = 0;
    uint8_t stream_channels_ = 0;
    uint8_t stream_bits_per_sample_ = 0;
    bool stream_little_endian_ = false;
    static constexpr size_t kPcmBufferSamples = 16000;
    int16_t pcm_buffer_[kPcmBufferSamples] = {};
    size_t pcm_write_idx_ = 0;
    size_t pcm_read_idx_ = 0;
    size_t pcm_available_ = 0;
};
