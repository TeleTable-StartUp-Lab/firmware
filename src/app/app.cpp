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

    RobotState state;
    OledUi oled(state, sensors);
    ConsoleCommander console(drive, sensors, leds, audio);
    BackendCoordinator backend(state, drive, sensors, leds, audio);

    void statusPrintTask(uint32_t nowMs)
    {
        if (nowMs - lastStatusPrintMs < 2000)
            return;

        lastStatusPrintMs = nowMs;

        Serial.printf("[status] wifi=%s ip=%s ws=%s mode=%s obs_front=%d batt=%d%% voltage=%.2fV current=%.2fA\n",
                      WifiManager::isConnected() ? "ok" : "no",
                      WifiManager::ip().c_str(),
                      WsControlClient::isConnected() ? "ok" : "no",
                      (state.driveMode() == RobotHttpServer::DriveMode::IDLE) ? "IDLE" : (state.driveMode() == RobotHttpServer::DriveMode::MANUAL) ? "MANUAL"
                                                                                                                                                   : "AUTO",
                      drive.obstacleFrontActive() ? 1 : 0,
                      sensors.batteryLevel(),
                      static_cast<double>(sensors.batteryVoltage()),
                      static_cast<double>(sensors.batteryCurrentA()));
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
        BackendClient::begin();

        const uint32_t bootMs = millis();
        drive.begin(bootMs);

        console.begin();

        sensors.beginIr();

        Wire.begin(static_cast<int>(BoardPins::I2C_SDA), static_cast<int>(BoardPins::I2C_SCL));
        Wire.setClock(100000);
        delay(50);

        scanI2C();

        const bool dok = oled.begin();
        Serial.printf("[oled] init %s (addr=0x%02X)\n", dok ? "ok" : "fail", BoardPins::OLED_I2C_ADDRESS);
        if (dok)
            oled.bootScreen();

        const bool lok = sensors.beginLux();
        Serial.printf("[bh1750] init %s (addr=0x%02X)\n", lok ? "ok" : "fail", BoardPins::BH1750_I2C_ADDRESS);

        const bool pok = sensors.beginPowerMonitor();
        Serial.printf("[ina226] init %s (addr=0x%02X)\n", pok ? "ok" : "fail", BoardPins::INA226_I2C_ADDRESS);

        const bool mok = sensors.beginImu();
        Serial.printf("[mpu6050] init %s (addr=0x%02X)\n", mok ? "ok" : "fail", sensors.imuAddress());

        const bool rok = sensors.beginRfid();
        Serial.printf("[rc522] init %s (ver=0x%02X ss=%d rst=%d sck=%d miso=%d mosi=%d)\n",
                      rok ? "ok" : "fail",
                      sensors.rfidVersion(),
                      static_cast<int>(BoardPins::RC522_SDA_SS),
                      static_cast<int>(BoardPins::RC522_RST),
                      static_cast<int>(BoardPins::RC522_SCK),
                      static_cast<int>(BoardPins::RC522_MISO),
                      static_cast<int>(BoardPins::RC522_MOSI));

        leds.begin();

        const bool aok = audio.begin();
        Serial.printf("[audio] init %s\n", aok ? "ok" : "fail");
        audio.setVolume(0.20f);

        const bool wok = WifiManager::begin(Secrets::WIFI_SSID, Secrets::WIFI_PASS, 15000);
        Serial.printf("[wifi] %s ip=%s\n", wok ? "connected" : "failed", WifiManager::ip().c_str());

        backend.begin();

        if (wok)
        {
            backend.registerTask(millis());
            backend.pushState();
        }

        Serial.println("[boot] motors + IR(front) + BH1750 + INA226 + MPU-6050 + RC522 + WS2812B + I2S audio + OLED + backend ws/http");
        Serial.println("[boot] obstacle policy: blocks FORWARD, reverse allowed");
        console.printHelp();
    }

    void loop()
    {
        const uint32_t nowMs = millis();

        backend.handle();

        statusPrintTask(nowMs);

        sensors.update(nowMs);
        oled.update(nowMs);
        leds.autoTask();

        console.handle();
        drive.update(nowMs, state.driveMode());

        backend.registerTask(nowMs);
        backend.stateTask(nowMs);
        backend.eventTask(nowMs);

        delay(1);
    }
}
