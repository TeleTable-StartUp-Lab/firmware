#pragma once

#include <Arduino.h>
#include <Wire.h>

class Mpu6050Sensor
{
public:
    struct Config
    {
        TwoWire *wire;
        uint8_t address;
        uint32_t sample_period_ms;
    };

    struct Reading
    {
        int16_t accel_x_raw = 0;
        int16_t accel_y_raw = 0;
        int16_t accel_z_raw = 0;
        int16_t temp_raw = 0;
        int16_t gyro_x_raw = 0;
        int16_t gyro_y_raw = 0;
        int16_t gyro_z_raw = 0;

        float accel_x_g = 0.0f;
        float accel_y_g = 0.0f;
        float accel_z_g = 0.0f;
        float temperature_c = 0.0f;
        float gyro_x_dps = 0.0f;
        float gyro_y_dps = 0.0f;
        float gyro_z_dps = 0.0f;
    };

    explicit Mpu6050Sensor(const Config &cfg);

    bool begin();
    void update(uint32_t now_ms);

    void setAddress(uint8_t address);
    uint8_t address() const;

    bool isOk() const;
    bool hasReading() const;
    const Reading &reading() const;

private:
    Config cfg_;
    bool ok_ = false;
    bool has_reading_ = false;
    uint32_t last_sample_ms_ = 0;
    Reading reading_;

    bool writeRegister_(uint8_t reg, uint8_t value);
    bool readRegisters_(uint8_t start_reg, uint8_t *buffer, size_t len);
    bool readSample_();
};
