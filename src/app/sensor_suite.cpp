#include "app/sensor_suite.h"

#include "board_pins.h"

#include <SPI.h>
#include <Wire.h>

SensorSuite::SensorSuite()
    : irLeft({.pin = BoardPins::IR_LEFT, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false}),
      irMid({.pin = BoardPins::IR_MIDDLE, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false}),
      irRight({.pin = BoardPins::IR_RIGHT, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false}),
      lightSensor({.wire = &Wire, .address = BoardPins::BH1750_I2C_ADDRESS, .sample_period_ms = 250}),
      powerSensor({.wire = &Wire,
                   .address = BoardPins::INA226_I2C_ADDRESS,
                   .sample_period_ms = 250,
                   .shunt_ohms = BoardPins::INA226_SHUNT_OHMS,
                   .battery_empty_voltage = BoardPins::BATTERY_EMPTY_VOLTAGE,
                   .battery_full_voltage = BoardPins::BATTERY_FULL_VOLTAGE}),
      imuSensor({.wire = &Wire, .address = BoardPins::MPU6050_I2C_ADDRESS, .sample_period_ms = 100}),
      rfidSensor({.spi = &SPI,
                  .sck_pin = BoardPins::RC522_SCK,
                  .miso_pin = BoardPins::RC522_MISO,
                  .mosi_pin = BoardPins::RC522_MOSI,
                  .ss_pin = BoardPins::RC522_SDA_SS,
                  .rst_pin = BoardPins::RC522_RST,
                  .poll_period_ms = 100}),
      irWatch(true),
      irPeriodic(false),
      luxPeriodic(false),
      imuPeriodic(false),
      powerPeriodic(false),
      rfidWatch(true),
      lastIrPrintMs(0),
      lastLuxPrintMs(0),
      lastImuPrintMs(0),
      lastPowerPrintMs(0)
{
}

void SensorSuite::beginIr()
{
    irLeft.begin();
    irMid.begin();
    irRight.begin();
}

bool SensorSuite::beginLux()
{
    return lightSensor.begin();
}

bool SensorSuite::beginPowerMonitor()
{
    return powerSensor.begin();
}

bool SensorSuite::beginImu()
{
    imuSensor.setAddress(BoardPins::MPU6050_I2C_ADDRESS);
    if (imuSensor.begin())
        return true;

    imuSensor.setAddress(BoardPins::MPU6050_I2C_ADDRESS_ALT);
    return imuSensor.begin();
}

bool SensorSuite::beginRfid()
{
    return rfidSensor.begin();
}

void SensorSuite::update(uint32_t nowMs)
{
    irLeft.update(nowMs);
    irMid.update(nowMs);
    irRight.update(nowMs);

    if (irWatch)
    {
        if (irLeft.roseObstacle())
            Serial.println("[ir] left obstacle");
        if (irLeft.fellObstacle())
            Serial.println("[ir] left clear");

        if (irMid.roseObstacle())
            Serial.println("[ir] middle obstacle");
        if (irMid.fellObstacle())
            Serial.println("[ir] middle clear");

        if (irRight.roseObstacle())
            Serial.println("[ir] right obstacle");
        if (irRight.fellObstacle())
            Serial.println("[ir] right clear");
    }

    if (irPeriodic && (nowMs - lastIrPrintMs >= 500))
    {
        lastIrPrintMs = nowMs;
        printIrOnce();
    }

    lightSensor.update(nowMs);
    powerSensor.update(nowMs);
    imuSensor.update(nowMs);

    if (rfidSensor.update(nowMs) && rfidWatch)
        printRfidOnce();

    if (luxPeriodic && (nowMs - lastLuxPrintMs >= 500))
    {
        lastLuxPrintMs = nowMs;
        printLuxOnce();
    }

    if (imuPeriodic && (nowMs - lastImuPrintMs >= 500))
    {
        lastImuPrintMs = nowMs;
        printImuOnce();
    }

    if (powerPeriodic && (nowMs - lastPowerPrintMs >= 500))
    {
        lastPowerPrintMs = nowMs;
        printPowerOnce();
    }
}

void SensorSuite::printIrOnce()
{
    Serial.printf("[ir] L=%d M=%d R=%d (1=obstacle)\n",
                  irLeft.isObstacle() ? 1 : 0,
                  irMid.isObstacle() ? 1 : 0,
                  irRight.isObstacle() ? 1 : 0);
}

void SensorSuite::printLuxOnce()
{
    if (!lightSensor.hasReading())
    {
        Serial.println("[lux] no reading");
        return;
    }
    Serial.printf("[lux] %.1f lx\n", static_cast<double>(lightSensor.lux()));
}

void SensorSuite::printImuOnce() const
{
    if (!imuSensor.hasReading())
    {
        Serial.println("[mpu6050] no reading");
        return;
    }

    const auto &reading = imuSensor.reading();
    Serial.printf("[mpu6050] accel=%.2fg %.2fg %.2fg gyro=%.2fdps %.2fdps %.2fdps temp=%.2fC\n",
                  static_cast<double>(reading.accel_x_g),
                  static_cast<double>(reading.accel_y_g),
                  static_cast<double>(reading.accel_z_g),
                  static_cast<double>(reading.gyro_x_dps),
                  static_cast<double>(reading.gyro_y_dps),
                  static_cast<double>(reading.gyro_z_dps),
                  static_cast<double>(reading.temperature_c));
}

void SensorSuite::printPowerOnce() const
{
    if (!powerSensor.hasReading())
    {
        Serial.println("[ina226] no reading");
        return;
    }

    const auto &reading = powerSensor.reading();
    Serial.printf("[ina226] battery=%d%% bus=%.3fV current=%.3fA power=%.3fW shunt=%.3fmV\n",
                  reading.battery_percent,
                  static_cast<double>(reading.bus_voltage_v),
                  static_cast<double>(reading.current_a),
                  static_cast<double>(reading.power_w),
                  static_cast<double>(reading.shunt_voltage_mv));
}

void SensorSuite::printRfidOnce() const
{
    if (!rfidSensor.isOk())
    {
        Serial.println("[rfid] reader offline");
        return;
    }

    if (!rfidSensor.hasReading())
    {
        Serial.println("[rfid] no card read yet");
        return;
    }

    const auto &card = rfidSensor.reading();
    Serial.printf("[rfid] uid=%s type=%s sak=0x%02X uid_bytes=%u\n",
                  card.uid_hex.c_str(),
                  card.card_type.c_str(),
                  card.sak,
                  card.uid_size);
}

void SensorSuite::setIrWatch(bool on)
{
    irWatch = on;
}

void SensorSuite::setIrPeriodic(bool on)
{
    irPeriodic = on;
}

void SensorSuite::setLuxPeriodic(bool on)
{
    luxPeriodic = on;
}

void SensorSuite::setImuPeriodic(bool on)
{
    imuPeriodic = on;
}

void SensorSuite::setPowerPeriodic(bool on)
{
    powerPeriodic = on;
}

void SensorSuite::setRfidWatch(bool on)
{
    rfidWatch = on;
}

bool SensorSuite::frontObstacleNow() const
{
    return irLeft.isObstacle() || irMid.isObstacle() || irRight.isObstacle();
}

bool SensorSuite::isIrLeftObstacle() const
{
    return irLeft.isObstacle();
}

bool SensorSuite::isIrMidObstacle() const
{
    return irMid.isObstacle();
}

bool SensorSuite::isIrRightObstacle() const
{
    return irRight.isObstacle();
}

bool SensorSuite::hasLux() const
{
    return lightSensor.hasReading();
}

float SensorSuite::lux() const
{
    return lightSensor.lux();
}

bool SensorSuite::hasPowerMonitor() const
{
    return powerSensor.hasReading();
}

int SensorSuite::batteryLevel() const
{
    return powerSensor.hasReading() ? powerSensor.reading().battery_percent : 0;
}

float SensorSuite::batteryVoltage() const
{
    return powerSensor.hasReading() ? powerSensor.reading().bus_voltage_v : 0.0f;
}

float SensorSuite::batteryCurrentA() const
{
    return powerSensor.hasReading() ? powerSensor.reading().current_a : 0.0f;
}

float SensorSuite::batteryPowerW() const
{
    return powerSensor.hasReading() ? powerSensor.reading().power_w : 0.0f;
}

const Ina226Sensor::Reading &SensorSuite::powerReading() const
{
    return powerSensor.reading();
}

bool SensorSuite::hasImu() const
{
    return imuSensor.hasReading();
}

uint8_t SensorSuite::imuAddress() const
{
    return imuSensor.address();
}

const Mpu6050Sensor::Reading &SensorSuite::imu() const
{
    return imuSensor.reading();
}

bool SensorSuite::hasRfid() const
{
    return rfidSensor.hasReading();
}

uint8_t SensorSuite::rfidVersion() const
{
    return rfidSensor.version();
}

const RfidRc522Sensor::Reading &SensorSuite::rfid() const
{
    return rfidSensor.reading();
}
