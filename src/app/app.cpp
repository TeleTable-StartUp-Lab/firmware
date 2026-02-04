#include "app/app.h"

#include "app/backend_coordinator.h"
#include "app/console_commander.h"
#include "app/drive_controller.h"
#include "app/led_controller.h"
#include "app/oled_ui.h"
#include "app/robot_state.h"
#include "app/sensor_suite.h"
#include "app_config.h"
#include "board_pins.h"
#include "secrets.h"

#include "drivers/i2s_audio.h"

#include "net/backend_client.h"
#include "net/backend_config.h"
#include "net/wifi_manager.h"
#include "net/ws_control_client.h"

#include <Wire.h>

namespace
{
    uint32_t lastStatusPrintMs = 0;

    SensorSuite sensors;
    DriveController drive(sensors);
    LedController leds(sensors);

    I2sAudio audio({.bclk_pin = static_cast<int>(BoardPins::I2S_BCLK),
                    .lrclk_pin = static_cast<int>(BoardPins::I2S_LRCLK),
                    .dout_pin = static_cast<int>(BoardPins::I2S_DOUT),
                    .sample_rate_hz = 22050});

    OledUi oled(sensors, leds, drive);
    ConsoleCommander console(drive, sensors, leds, audio);

    RobotState state;
    BackendCoordinator backend(state, drive, sensors, leds, audio);

    void statusPrintTask(uint32_t nowMs)
    {
        if (nowMs - lastStatusPrintMs < 2000)
            return;

        lastStatusPrintMs = nowMs;

        Serial.printf("[status] wifi=%s ip=%s ws=%s mode=%s obs_front=%d\n",
                      WifiManager::isConnected() ? "ok" : "no",
                      WifiManager::ip().c_str(),
                      WsControlClient::isConnected() ? "ok" : "no",
                      (state.driveMode() == RobotHttpServer::DriveMode::IDLE) ? "IDLE" : (state.driveMode() == RobotHttpServer::DriveMode::MANUAL) ? "MANUAL"
                                                                                                                                                   : "AUTO",
                      drive.obstacleFrontActive() ? 1 : 0);
    }

    void scanI2C()
    {
        Serial.println("[i2c] scanning...");
        uint8_t count = 0;
        for (uint8_t addr = 1; addr < 127; addr++)
        {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0)
            {
                Serial.printf("[i2c] found 0x%02X\n", addr);
                count++;
            }
        }
        if (!count)
            Serial.println("[i2c] no devices found");
    }
}

namespace App
{
    void setup()
    {
        Serial.begin(AppConfig::SERIAL_BAUD);
        delay(50);

        const uint32_t bootMs = millis();
        drive.begin(bootMs);

        console.begin();

        sensors.beginIr();

        Wire.begin(static_cast<int>(BoardPins::I2C_SDA), static_cast<int>(BoardPins::I2C_SCL));
        Wire.setClock(100000);
        delay(50);

        scanI2C();

        const bool dok = oled.begin();
        Serial.printf("[oled] init %s (addr=0x%02X)\n", dok ? "ok" : "fail", 0x3C);
        if (dok)
            oled.bootScreen();

        const bool lok = sensors.beginLux();
        Serial.printf("[bh1750] init %s (addr=0x%02X)\n", lok ? "ok" : "fail", 0x23);

        leds.begin();

        const bool aok = audio.begin();
        Serial.printf("[audio] init %s\n", aok ? "ok" : "fail");
        audio.setVolume(0.20f);

        const bool wok = WifiManager::begin(Secrets::WIFI_SSID, Secrets::WIFI_PASS, 15000);
        Serial.printf("[wifi] %s ip=%s\n", wok ? "connected" : "failed", WifiManager::ip().c_str());

        if (wok)
        {
            const bool regOk = BackendClient::registerRobot(BackendConfig::ROBOT_PORT);
            Serial.printf("[backend] register %s\n", regOk ? "ok" : "fail");
            backend.pushState();
        }

        backend.begin();

        Serial.println("[boot] motors + IR(front) + BH1750 + WS2812B + I2S audio + OLED + backend ws/http");
        Serial.println("[boot] obstacle policy: blocks FORWARD, reverse allowed");
        console.printHelp();
    }

    void loop()
    {
        const uint32_t nowMs = millis();

        statusPrintTask(nowMs);

        sensors.update(nowMs);
        oled.update(nowMs);
        leds.autoTask();

        console.handle();
        drive.update(nowMs, state.driveMode());

        backend.handle();

        backend.registerTask(nowMs);
        backend.stateTask(nowMs);

        delay(1);
    }
}
