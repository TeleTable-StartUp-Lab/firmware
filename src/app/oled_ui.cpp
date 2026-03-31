#include "app/oled_ui.h"

#include "board_pins.h"
#include "net/wifi_manager.h"
#include "net/ws_control_client.h"

#include <Wire.h>

namespace
{
    constexpr uint8_t OLED_WIDTH = 128;
    constexpr uint8_t OLED_HEIGHT = 32;
    constexpr uint8_t OLED_LINE_COUNT = OLED_HEIGHT / 8;
    constexpr uint32_t OLED_RECOVERY_INTERVAL_MS = 5000;
}

OledUi::OledUi(RobotState &stateRef, SensorSuite &sensorsRef)
    : oled({.wire = &Wire, .address = BoardPins::OLED_I2C_ADDRESS, .width = OLED_WIDTH, .height = OLED_HEIGHT, .reset_pin = BoardPins::OLED_RESET_PIN}),
      state(stateRef),
      sensors(sensorsRef),
      lastOledMs(0),
      lastRecoveryAttemptMs(0)
{
}

bool OledUi::begin()
{
    lastRecoveryAttemptMs = millis();
    return oled.begin();
}

void OledUi::bootScreen()
{
    String l[OLED_LINE_COUNT];
    l[0] = "Firmware Teletable";
    l[1] = "I2C: OLED   0x" + String(BoardPins::OLED_I2C_ADDRESS, HEX);
    l[2] = "I2C: INA226 0x" + String(BoardPins::INA226_I2C_ADDRESS, HEX);
    l[3] = "Booting...";
    oled.setLines(l, OLED_LINE_COUNT);
    oled.show();
}

void OledUi::update(uint32_t nowMs)
{
    if (!oled.isOk())
    {
        if (nowMs - lastRecoveryAttemptMs >= OLED_RECOVERY_INTERVAL_MS)
        {
            lastRecoveryAttemptMs = nowMs;
            const bool recovered = recover();
            Serial.printf("[oled] recovery %s (present=%d addr=0x%02X)\n",
                          recovered ? "ok" : "fail",
                          probe() ? 1 : 0,
                          BoardPins::OLED_I2C_ADDRESS);
        }
        return;
    }

    if (nowMs - lastOledMs < 500)
        return;
    lastOledMs = nowMs;

    String l[OLED_LINE_COUNT];
    if (sensors.hasPowerMonitor())
    {
        const auto &power = sensors.powerReading();
        l[0] = "Akku: " + String(power.battery_percent) + "% " + String(power.bus_voltage_v, 2) + "V";
        l[1] = "Verbr: " + String(power.current_a, 2) + "A " + String(power.power_w, 2) + "W";
    }
    else
    {
        l[0] = "Akku: kein INA226";
        l[1] = "Verbr: keine Daten";
    }

    l[2] = String("WS: ") + (WsControlClient::isConnected() ? "online" : "offline");
    l[3] = String("State: ") + state.driveModeToBackend();

    oled.setLines(l, OLED_LINE_COUNT);
    oled.show();
}

bool OledUi::isOk() const
{
    return oled.isOk();
}

bool OledUi::probe() const
{
    return oled.probe();
}

bool OledUi::recover()
{
    lastRecoveryAttemptMs = millis();

    const bool ok = oled.recover();
    if (ok)
    {
        lastOledMs = 0;
        bootScreen();
    }
    return ok;
}

void OledUi::clear()
{
    oled.clear();
}

void OledUi::showTestScreen()
{
    String l[OLED_LINE_COUNT];
    l[0] = "OLED diagnostic";
    l[1] = "Addr: 0x" + String(BoardPins::OLED_I2C_ADDRESS, HEX);
    l[2] = "INA226: 0x" + String(BoardPins::INA226_I2C_ADDRESS, HEX);
    l[3] = String("WiFi: ") + (WifiManager::isConnected() ? "ok" : "off");
    oled.setLines(l, OLED_LINE_COUNT);
    oled.show();
}
