#include "app/app.h"
#include "app_config.h"
#include "board_pins.h"
#include "app/serial_console.h"
#include "drivers/hbridge_motor.h"
#include "drivers/obstacle_sensor.h"
#include "drivers/bh1750_sensor.h"
#include "drivers/i2s_audio.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "net/wifi_manager.h"
#include "secrets.h"
#include "net/wifi_manager.h"
#include "net/backend_client.h"
#include "net/backend_config.h"
#include "secrets.h"
#include "net/robot_http_server.h"
#include "net/backend_config.h"

namespace
{
    uint32_t lastHeartbeatMs = 0;
    uint32_t lastStatusPrintMs = 0;
    uint32_t lastIrPrintMs = 0;
    uint32_t lastLuxPrintMs = 0;
    bool ledState = false;

    SerialConsole console;

    HBridgeMotor leftMotor({.in1 = BoardPins::LEFT_MOTOR_IN1,
                            .in2 = BoardPins::LEFT_MOTOR_IN2,
                            .pwm_hz = 20000,
                            .pwm_resolution_bits = 10});

    HBridgeMotor rightMotor({.in1 = BoardPins::RIGHT_MOTOR_IN1,
                             .in2 = BoardPins::RIGHT_MOTOR_IN2,
                             .pwm_hz = 20000,
                             .pwm_resolution_bits = 10});

    ObstacleSensor irLeft({.pin = BoardPins::IR_LEFT, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false});
    ObstacleSensor irMid({.pin = BoardPins::IR_MIDDLE, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false});
    ObstacleSensor irRight({.pin = BoardPins::IR_RIGHT, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false});

    bool irWatch = true;
    bool irPeriodic = false;

    Bh1750Sensor lightSensor({.wire = &Wire,
                              .address = 0x23, // change to 0x5C if ADDR is high
                              .sample_period_ms = 250});

    bool luxPeriodic = true;

    I2sAudio audio({.bclk_pin = static_cast<int>(BoardPins::I2S_BCLK),
                    .lrclk_pin = static_cast<int>(BoardPins::I2S_LRCLK),
                    .dout_pin = static_cast<int>(BoardPins::I2S_DOUT),
                    .sample_rate_hz = 22050});

    float clampf(float v, float lo, float hi)
    {
        if (v < lo)
            return lo;
        if (v > hi)
            return hi;
        return v;
    }

    void setTank(float throttle, float steer)
    {
        throttle = clampf(throttle, -1.0f, 1.0f);
        steer = clampf(steer, -1.0f, 1.0f);

        const float left = clampf(throttle + steer, -1.0f, 1.0f);
        const float right = clampf(throttle - steer, -1.0f, 1.0f);

        leftMotor.set(left);
        rightMotor.set(right);

        Serial.printf("[tank] throttle=%.3f steer=%.3f => L=%.3f R=%.3f\n",
                      static_cast<double>(throttle),
                      static_cast<double>(steer),
                      static_cast<double>(left),
                      static_cast<double>(right));
    }

    void printIrOnce()
    {
        Serial.printf("[ir] L=%d M=%d R=%d (1=obstacle)\n",
                      irLeft.isObstacle() ? 1 : 0,
                      irMid.isObstacle() ? 1 : 0,
                      irRight.isObstacle() ? 1 : 0);
    }

    void printLuxOnce()
    {
        if (!lightSensor.hasReading())
        {
            Serial.println("[lux] no reading");
            return;
        }
        Serial.printf("[lux] %.1f lx\n", static_cast<double>(lightSensor.lux()));
    }

    // ---------------- LED strip ----------------

    constexpr uint16_t LED_COUNT = 144;

    Adafruit_NeoPixel ledStrip(
        LED_COUNT,
        static_cast<uint8_t>(BoardPins::LED_STRIP_DATA),
        NEO_GRB + NEO_KHZ800);

    bool ledEnabled = false;
    bool ledAutoEnabled = true;

    uint8_t ledBrightness = 40;
    uint8_t ledR = 255;
    uint8_t ledG = 255;
    uint8_t ledB = 255;

    void ledApply()
    {
        if (!ledEnabled)
        {
            ledStrip.clear();
            ledStrip.show();
            return;
        }

        const uint32_t c = ledStrip.Color(ledR, ledG, ledB);
        for (uint16_t i = 0; i < LED_COUNT; ++i)
        {
            ledStrip.setPixelColor(i, c);
        }

        ledStrip.setBrightness(ledBrightness);
        ledStrip.show();
    }

    void ledSetEnabled(bool enabled)
    {
        if (ledEnabled == enabled)
            return;

        ledEnabled = enabled;
        ledApply();

        Serial.printf("[led] %s\n", ledEnabled ? "ON" : "OFF");
    }

    void ledAutoTask()
    {
        if (!ledAutoEnabled)
            return;

        if (!lightSensor.hasReading())
            return;

        const float lux = lightSensor.lux();

        if (!ledEnabled && lux < BoardPins::LED_LUX_ON_THRESHOLD)
        {
            ledSetEnabled(true);
            Serial.printf("[led] auto on (lux=%.1f)\n", static_cast<double>(lux));
        }
        else if (ledEnabled && lux > BoardPins::LED_LUX_OFF_THRESHOLD)
        {
            ledSetEnabled(false);
            Serial.printf("[led] auto off (lux=%.1f)\n", static_cast<double>(lux));
        }
    }

    // ------------- Console help / handling -------------

    void printHelp()
    {
        Serial.println("Commands:");
        Serial.println("  help                - show commands");
        Serial.println("  l <v>               - set left motor [-1.0..1.0]");
        Serial.println("  r <v>               - set right motor [-1.0..1.0]");
        Serial.println("  tank <t> <s>        - set throttle/steer [-1.0..1.0]");
        Serial.println("  stop                - stop both motors (coast)");
        Serial.println("  ir                  - print IR sensor states once");
        Serial.println("  irwatch on|off       - print IR edge events");
        Serial.println("  irperiodic on|off    - periodic IR snapshot every 500ms");
        Serial.println("  lux                 - print light sensor lux once");
        Serial.println("  luxperiodic on|off   - periodic lux print every 500ms");
        Serial.println("  led on|off          - manual LED on/off");
        Serial.println("  ledauto on|off      - enable/disable auto LED control");
        Serial.println("  ledcolor <r> <g> <b> - set LED color (0..255)");
        Serial.println("  ledbri <x>          - set LED brightness (0..255)");
        Serial.println("  vol <0..100>        - set audio volume percent");
        Serial.println("  beep                - play short test beep");
        Serial.println("  beep <hz> <ms>      - play beep with frequency and duration");
    }

    void handleConsole()
    {
        String line;
        if (!console.pollLine(line))
            return;

        if (line.equalsIgnoreCase("help"))
        {
            printHelp();
            return;
        }

        if (line.equalsIgnoreCase("stop"))
        {
            leftMotor.stop();
            rightMotor.stop();
            Serial.println("[motor] stop");
            return;
        }

        if (line.startsWith("l "))
        {
            const float v = line.substring(2).toFloat();
            leftMotor.set(v);
            Serial.printf("[motor] left=%.3f\n", static_cast<double>(v));
            return;
        }

        if (line.startsWith("r "))
        {
            const float v = line.substring(2).toFloat();
            rightMotor.set(v);
            Serial.printf("[motor] right=%.3f\n", static_cast<double>(v));
            return;
        }

        if (line.startsWith("tank "))
        {
            const int firstSpace = line.indexOf(' ');
            const int secondSpace = line.indexOf(' ', firstSpace + 1);
            if (secondSpace < 0)
            {
                Serial.println("[console] usage: tank <throttle> <steer>");
                return;
            }
            const float throttle = line.substring(firstSpace + 1, secondSpace).toFloat();
            const float steer = line.substring(secondSpace + 1).toFloat();
            setTank(throttle, steer);
            return;
        }

        if (line.equalsIgnoreCase("ir"))
        {
            printIrOnce();
            return;
        }
        if (line.equalsIgnoreCase("irwatch on"))
        {
            irWatch = true;
            Serial.println("[ir] watch on");
            return;
        }
        if (line.equalsIgnoreCase("irwatch off"))
        {
            irWatch = false;
            Serial.println("[ir] watch off");
            return;
        }
        if (line.equalsIgnoreCase("irperiodic on"))
        {
            irPeriodic = true;
            Serial.println("[ir] periodic on");
            return;
        }
        if (line.equalsIgnoreCase("irperiodic off"))
        {
            irPeriodic = false;
            Serial.println("[ir] periodic off");
            return;
        }

        if (line.equalsIgnoreCase("lux"))
        {
            printLuxOnce();
            return;
        }
        if (line.equalsIgnoreCase("luxperiodic on"))
        {
            luxPeriodic = true;
            Serial.println("[lux] periodic on");
            return;
        }
        if (line.equalsIgnoreCase("luxperiodic off"))
        {
            luxPeriodic = false;
            Serial.println("[lux] periodic off");
            return;
        }

        // --- LED commands ---

        if (line.equalsIgnoreCase("led on"))
        {
            ledAutoEnabled = false;
            ledSetEnabled(true);
            return;
        }

        if (line.equalsIgnoreCase("led off"))
        {
            ledAutoEnabled = false;
            ledSetEnabled(false);
            return;
        }

        if (line.equalsIgnoreCase("ledauto on"))
        {
            ledAutoEnabled = true;
            Serial.println("[led] auto on");
            return;
        }

        if (line.equalsIgnoreCase("ledauto off"))
        {
            ledAutoEnabled = false;
            Serial.println("[led] auto off");
            return;
        }

        if (line.startsWith("ledbri "))
        {
            long v = line.substring(7).toInt();
            if (v < 0)
                v = 0;
            if (v > 255)
                v = 255;

            ledBrightness = static_cast<uint8_t>(v);
            ledApply();
            Serial.printf("[led] brightness=%d\n", static_cast<int>(ledBrightness));
            return;
        }

        if (line.startsWith("ledcolor "))
        {
            const int s1 = line.indexOf(' ');
            const int s2 = line.indexOf(' ', s1 + 1);
            const int s3 = line.indexOf(' ', s2 + 1);
            if (s3 < 0)
            {
                Serial.println("[console] usage: ledcolor <r> <g> <b>");
                return;
            }

            long r = line.substring(s1 + 1, s2).toInt();
            long g = line.substring(s2 + 1, s3).toInt();
            long b = line.substring(s3 + 1).toInt();

            if (r < 0)
                r = 0;
            if (r > 255)
                r = 255;
            if (g < 0)
                g = 0;
            if (g > 255)
                g = 255;
            if (b < 0)
                b = 0;
            if (b > 255)
                b = 255;

            ledR = static_cast<uint8_t>(r);
            ledG = static_cast<uint8_t>(g);
            ledB = static_cast<uint8_t>(b);

            ledApply();
            Serial.printf("[led] color=%d,%d,%d\n", static_cast<int>(ledR), static_cast<int>(ledG), static_cast<int>(ledB));
            return;
        }

        // --- Audio commands ---

        if (line.startsWith("vol "))
        {
            long p = line.substring(4).toInt();
            if (p < 0)
                p = 0;
            if (p > 100)
                p = 100;

            audio.setVolume(static_cast<float>(p) / 100.0f);
            Serial.printf("[audio] volume=%ld%%\n", p);
            return;
        }

        if (line.equalsIgnoreCase("beep"))
        {
            audio.playBeep(880, 150);
            return;
        }

        if (line.startsWith("beep "))
        {
            const int s1 = line.indexOf(' ');
            const int s2 = line.indexOf(' ', s1 + 1);
            if (s2 < 0)
            {
                Serial.println("[console] usage: beep <hz> <ms>");
                return;
            }

            long hz = line.substring(s1 + 1, s2).toInt();
            long ms = line.substring(s2 + 1).toInt();

            if (hz < 50)
                hz = 50;
            if (hz > 5000)
                hz = 5000;
            if (ms < 10)
                ms = 10;
            if (ms > 2000)
                ms = 2000;

            audio.playBeep(static_cast<uint16_t>(hz), static_cast<uint16_t>(ms));
            return;
        }

        Serial.println("[console] unknown command (type: help)");
    }

    void heartbeatTask(uint32_t nowMs)
    {
        if (nowMs - lastHeartbeatMs < AppConfig::HEARTBEAT_PERIOD_MS)
            return;
        lastHeartbeatMs = nowMs;

        ledState = !ledState;
        digitalWrite(static_cast<uint8_t>(BoardPins::HEARTBEAT_LED), ledState ? HIGH : LOW);
    }

    void statusPrintTask(uint32_t nowMs)
    {
        if (nowMs - lastStatusPrintMs < AppConfig::STATUS_PRINT_PERIOD_MS)
            return;
        lastStatusPrintMs = nowMs;

        Serial.printf("[status] uptime=%lu ms, heap_free=%u\n",
                      static_cast<unsigned long>(nowMs),
                      static_cast<unsigned int>(ESP.getFreeHeap()));
    }

    void irUpdateTask(uint32_t nowMs)
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
    }

    void luxUpdateTask(uint32_t nowMs)
    {
        lightSensor.update(nowMs);

        if (luxPeriodic && (nowMs - lastLuxPrintMs >= 500))
        {
            lastLuxPrintMs = nowMs;
            printLuxOnce();
        }
    }

    RobotHttpServer::DriveMode driveMode = RobotHttpServer::DriveMode::IDLE;

    String lastRouteStart;
    String lastRouteEnd;
    String positionStr = "";

}

namespace App
{
    void setup()
    {
        Serial.begin(AppConfig::SERIAL_BAUD);
        delay(50);

        pinMode(static_cast<uint8_t>(BoardPins::HEARTBEAT_LED), OUTPUT);
        digitalWrite(static_cast<uint8_t>(BoardPins::HEARTBEAT_LED), LOW);

        console.begin();
        leftMotor.begin();
        rightMotor.begin();

        irLeft.begin();
        irMid.begin();
        irRight.begin();

        Wire.begin(static_cast<int>(BoardPins::I2C_SDA), static_cast<int>(BoardPins::I2C_SCL));

        const bool ok = lightSensor.begin();
        Serial.printf("[bh1750] init %s (addr=0x%02X)\n", ok ? "ok" : "fail", 0x23);

        ledStrip.begin();
        ledStrip.clear();
        ledStrip.setBrightness(ledBrightness);
        ledStrip.show();

        const bool aok = audio.begin();
        Serial.printf("[audio] init %s\n", aok ? "ok" : "fail");
        audio.setVolume(0.20f);

        const bool wok = WifiManager::begin(Secrets::WIFI_SSID, Secrets::WIFI_PASS, 15000);
        Serial.printf("[wifi] %s ip=%s\n", wok ? "connected" : "failed", WifiManager::ip().c_str());

        if (wok)
        {
            const bool regOk = BackendClient::registerRobot(BackendConfig::ROBOT_PORT);
            Serial.printf("[backend] register %s\n", regOk ? "ok" : "fail");
        }
        RobotHttpServer::begin(
            BackendConfig::ROBOT_PORT,
            []() -> RobotHttpServer::StatusSnapshot
            {
                RobotHttpServer::StatusSnapshot s{};
                s.systemHealth = "OK";
                s.batteryLevel = 85; // TODO: real battery later
                s.driveMode = driveMode;
                s.cargoStatus = "IN_TRANSIT";

                s.lastRouteStart = lastRouteStart.length() ? lastRouteStart.c_str() : nullptr;
                s.lastRouteEnd = lastRouteEnd.length() ? lastRouteEnd.c_str() : nullptr;
                s.position = positionStr.length() ? positionStr.c_str() : nullptr;

                s.irLeft = irLeft.isObstacle();
                s.irMid = irMid.isObstacle();
                s.irRight = irRight.isObstacle();

                s.luxValid = lightSensor.hasReading();
                s.lux = s.luxValid ? lightSensor.lux() : 0.0f;

                s.ledEnabled = ledEnabled;
                s.ledAutoEnabled = ledAutoEnabled;

                s.audioVolume = audio.volume();
                return s;
            },
            [](RobotHttpServer::DriveMode m)
            {
                driveMode = m;

                if (driveMode == RobotHttpServer::DriveMode::IDLE)
                {
                    leftMotor.stop();
                    rightMotor.stop();
                }

                Serial.printf("[mode] set to %s\n",
                              (driveMode == RobotHttpServer::DriveMode::IDLE) ? "IDLE" : (driveMode == RobotHttpServer::DriveMode::MANUAL) ? "MANUAL"
                                                                                                                                           : "AUTO");
            },
            [](const String &startNode, const String &endNode)
            {
                lastRouteStart = startNode;
                lastRouteEnd = endNode;
                Serial.printf("[route] selected %s -> %s\n", startNode.c_str(), endNode.c_str());
            });

        Serial.println("[boot] base scaffold + motors + IR + BH1750 + WS2812B + I2S audio");
        printHelp();
    }

    void loop()
    {
        const uint32_t nowMs = millis();

        heartbeatTask(nowMs);
        statusPrintTask(nowMs);
        irUpdateTask(nowMs);
        luxUpdateTask(nowMs);
        ledAutoTask();
        handleConsole();
        RobotHttpServer::handle();

        delay(1);
    }
}
