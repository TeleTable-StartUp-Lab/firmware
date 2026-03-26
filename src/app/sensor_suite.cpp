#include "app/sensor_suite.h"

#include "board_pins.h"

#include <SPI.h>
#include <Wire.h>

SensorSuite::SensorSuite()
    : irLeft({.pin = BoardPins::IR_LEFT, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false}),
      irMid({.pin = BoardPins::IR_MIDDLE, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false}),
      irRight({.pin = BoardPins::IR_RIGHT, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false}),
      lightSensor({.wire = &Wire, .address = BoardPins::BH1750_I2C_ADDRESS, .sample_period_ms = 250}),
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
      luxPeriodic(true),
      rfidWatch(true),
      lastIrPrintMs(0),
      lastLuxPrintMs(0)
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
    imuSensor.update(nowMs);

    if (rfidSensor.update(nowMs) && rfidWatch)
        printRfidOnce();

    if (luxPeriodic && (nowMs - lastLuxPrintMs >= 500))
    {
        lastLuxPrintMs = nowMs;
        printLuxOnce();
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
