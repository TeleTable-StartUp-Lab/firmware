#include "app/oled_ui.h"

#include "net/wifi_manager.h"
#include "net/ws_control_client.h"

#include <Wire.h>

OledUi::OledUi(SensorSuite &sensorsRef, LedController &ledsRef, DriveController &driveRef)
    : oled({.wire = &Wire, .address = 0x3C, .width = 128, .height = 64, .reset_pin = -1}),
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
    l[1] = "I2C: BH1750 0x23";
    l[2] = "I2C: OLED   0x3C";
    l[3] = "Booting...";
    l[4] = "";
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

    if (drive.obstacleFrontActive())
        l[5] = "IR: FRONT BLOCK";
    else
        l[5] = "";

    oled.setLines(l, 6);
    oled.show();
}

bool OledUi::isOk() const
{
    return oled.isOk();
}
