#include "app/oled_ui.h"

#include "board_pins.h"
#include "net/wifi_manager.h"
#include "net/ws_control_client.h"

#include <Wire.h>

OledUi::OledUi(SensorSuite &sensorsRef, LedController &ledsRef, DriveController &driveRef)
    : oled({.wire = &Wire, .address = BoardPins::OLED_I2C_ADDRESS, .width = 128, .height = 64, .reset_pin = BoardPins::OLED_RESET_PIN}),
      sensors(sensorsRef),
      leds(ledsRef),
      drive(driveRef),
      lastOledMs(0)
{
}

bool OledUi::begin()
{
    return oled.begin();
}

void OledUi::bootScreen()
{
    String l[6];
    l[0] = "Firmware Teletable";
    l[1] = "I2C: OLED   0x" + String(BoardPins::OLED_I2C_ADDRESS, HEX);
    l[2] = "I2C: BH1750 0x" + String(BoardPins::BH1750_I2C_ADDRESS, HEX);
    l[3] = "I2C: MPU   0x" + String(BoardPins::MPU6050_I2C_ADDRESS, HEX);
    l[4] = "Booting...";
    l[5] = "";
    oled.setLines(l, 6);
    oled.show();
}

void OledUi::update(uint32_t nowMs)
{
    if (!oled.isOk())
        return;

    if (nowMs - lastOledMs < 500)
        return;
    lastOledMs = nowMs;

    String l[6];

    if (sensors.hasImu())
    {
        const auto &imu = sensors.imu();
        const uint8_t page = static_cast<uint8_t>((nowMs / 2500U) % 2U);

        if (page == 0)
        {
            l[0] = "MPU6050 Accel";
            l[1] = "Ax:" + String(imu.accel_x_g, 2) + " g";
            l[2] = "Ay:" + String(imu.accel_y_g, 2) + " g";
            l[3] = "Az:" + String(imu.accel_z_g, 2) + " g";
            l[4] = "T:" + String(imu.temperature_c, 1) + " C";
            l[5] = sensors.hasLux() ? ("Lux:" + String(sensors.lux(), 1)) : "Lux:n/a";
        }
        else
        {
            l[0] = "MPU6050 Gyro";
            l[1] = "Gx:" + String(imu.gyro_x_dps, 1) + " d/s";
            l[2] = "Gy:" + String(imu.gyro_y_dps, 1) + " d/s";
            l[3] = "Gz:" + String(imu.gyro_z_dps, 1) + " d/s";
            l[4] = String("WiFi:") + (WifiManager::isConnected() ? "ok" : "off");
            l[5] = drive.obstacleFrontActive() ? "IR: FRONT BLOCK" : (String("WS:") + (WsControlClient::isConnected() ? "ok" : "off"));
        }
    }
    else
    {
        l[0] = "Firmware Teletable";

        if (WifiManager::isConnected())
            l[1] = "WiFi: " + WifiManager::ip();
        else
            l[1] = "WiFi: disconnected";

        if (sensors.hasLux())
            l[2] = "Lux: " + String(sensors.lux(), 1);
        else
            l[2] = "Lux: n/a";

        l[3] = String("LED: ") + (leds.isEnabled() ? "on" : "off") + (leds.isAutoEnabled() ? " (A)" : " (M)");
        l[4] = String("WS: ") + (WsControlClient::isConnected() ? "connected" : "offline");
        l[5] = drive.obstacleFrontActive() ? "IR: FRONT BLOCK" : "";
    }

    oled.setLines(l, 6);
    oled.show();
}

bool OledUi::isOk() const
{
    return oled.isOk();
}
