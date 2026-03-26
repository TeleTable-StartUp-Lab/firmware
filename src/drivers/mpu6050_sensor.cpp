#include "drivers/mpu6050_sensor.h"

namespace
{
    constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    constexpr uint8_t REG_SMPLRT_DIV = 0x19;
    constexpr uint8_t REG_CONFIG = 0x1A;
    constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
    constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

    constexpr float ACCEL_LSB_PER_G = 16384.0f;   // +/-2 g
    constexpr float GYRO_LSB_PER_DPS = 131.0f;    // +/-250 deg/s
    constexpr float TEMP_LSB_PER_C = 340.0f;
    constexpr float TEMP_OFFSET_C = 36.53f;
}

Mpu6050Sensor::Mpu6050Sensor(const Config &cfg) : cfg_(cfg) {}

bool Mpu6050Sensor::begin()
{
    if (!cfg_.wire)
        return false;

    cfg_.wire->beginTransmission(cfg_.address);
    if (cfg_.wire->endTransmission() != 0)
    {
        ok_ = false;
        return false;
    }

    ok_ = writeRegister_(REG_PWR_MGMT_1, 0x00) &&      // wake up
          writeRegister_(REG_SMPLRT_DIV, 0x07) &&      // 1 kHz / (1 + 7) = 125 Hz
          writeRegister_(REG_CONFIG, 0x03) &&          // low-pass filter
          writeRegister_(REG_GYRO_CONFIG, 0x00) &&     // +/-250 deg/s
          writeRegister_(REG_ACCEL_CONFIG, 0x00);      // +/-2 g

    if (!ok_)
        return false;

    delay(50);
    has_reading_ = readSample_();
    last_sample_ms_ = millis();
    return ok_;
}

void Mpu6050Sensor::update(uint32_t now_ms)
{
    if (!ok_)
        return;
    if (now_ms - last_sample_ms_ < cfg_.sample_period_ms)
        return;

    last_sample_ms_ = now_ms;
    has_reading_ = readSample_();
}

void Mpu6050Sensor::setAddress(uint8_t address)
{
    cfg_.address = address;
    ok_ = false;
    has_reading_ = false;
}

uint8_t Mpu6050Sensor::address() const { return cfg_.address; }
bool Mpu6050Sensor::isOk() const { return ok_; }
bool Mpu6050Sensor::hasReading() const { return has_reading_; }
const Mpu6050Sensor::Reading &Mpu6050Sensor::reading() const { return reading_; }

bool Mpu6050Sensor::writeRegister_(uint8_t reg, uint8_t value)
{
    cfg_.wire->beginTransmission(cfg_.address);
    cfg_.wire->write(reg);
    cfg_.wire->write(value);
    return cfg_.wire->endTransmission() == 0;
}

bool Mpu6050Sensor::readRegisters_(uint8_t start_reg, uint8_t *buffer, size_t len)
{
    cfg_.wire->beginTransmission(cfg_.address);
    cfg_.wire->write(start_reg);
    if (cfg_.wire->endTransmission(false) != 0)
        return false;

    const uint8_t requested = cfg_.wire->requestFrom(static_cast<int>(cfg_.address),
                                                     static_cast<int>(len));
    if (requested != len)
        return false;

    for (size_t i = 0; i < len; ++i)
        buffer[i] = static_cast<uint8_t>(cfg_.wire->read());

    return true;
}

bool Mpu6050Sensor::readSample_()
{
    uint8_t buffer[14] = {0};
    if (!readRegisters_(REG_ACCEL_XOUT_H, buffer, sizeof(buffer)))
        return false;

    auto toInt16 = [&](size_t index) -> int16_t
    {
        return static_cast<int16_t>((static_cast<uint16_t>(buffer[index]) << 8) |
                                    static_cast<uint16_t>(buffer[index + 1]));
    };

    reading_.accel_x_raw = toInt16(0);
    reading_.accel_y_raw = toInt16(2);
    reading_.accel_z_raw = toInt16(4);
    reading_.temp_raw = toInt16(6);
    reading_.gyro_x_raw = toInt16(8);
    reading_.gyro_y_raw = toInt16(10);
    reading_.gyro_z_raw = toInt16(12);

    reading_.accel_x_g = static_cast<float>(reading_.accel_x_raw) / ACCEL_LSB_PER_G;
    reading_.accel_y_g = static_cast<float>(reading_.accel_y_raw) / ACCEL_LSB_PER_G;
    reading_.accel_z_g = static_cast<float>(reading_.accel_z_raw) / ACCEL_LSB_PER_G;
    reading_.temperature_c = (static_cast<float>(reading_.temp_raw) / TEMP_LSB_PER_C) + TEMP_OFFSET_C;
    reading_.gyro_x_dps = static_cast<float>(reading_.gyro_x_raw) / GYRO_LSB_PER_DPS;
    reading_.gyro_y_dps = static_cast<float>(reading_.gyro_y_raw) / GYRO_LSB_PER_DPS;
    reading_.gyro_z_dps = static_cast<float>(reading_.gyro_z_raw) / GYRO_LSB_PER_DPS;

    return true;
}
