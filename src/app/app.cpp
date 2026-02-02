#include "app/app.h"
#include "app/serial_console.h"
#include "app_config.h"
#include "board_pins.h"
#include "secrets.h"

#include "drivers/bh1750_sensor.h"
#include "drivers/hbridge_motor.h"
#include "drivers/i2s_audio.h"
#include "drivers/obstacle_sensor.h"
#include "drivers/oled_display.h"

#include "net/backend_client.h"
#include "net/backend_config.h"
#include "net/robot_http_server.h"
#include "net/wifi_manager.h"
#include "net/ws_control_client.h"

#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <cmath>
#include <cstdint>

namespace
{
    // ==========================
    // General / app state
    // ==========================

    RobotHttpServer::DriveMode driveMode = RobotHttpServer::DriveMode::IDLE;

    uint32_t lastStatusPrintMs = 0;
    uint32_t lastIrPrintMs = 0;
    uint32_t lastLuxPrintMs = 0;

    uint32_t lastBackendRegisterMs = 0;
    uint32_t lastBackendStateMs = 0;

    SerialConsole console;

    // ==========================
    // Motor drivers
    // ==========================

    HBridgeMotor leftMotor({.in1 = BoardPins::LEFT_MOTOR_IN1,
                            .in2 = BoardPins::LEFT_MOTOR_IN2,
                            .pwm_hz = 20000,
                            .pwm_resolution_bits = 10});

    HBridgeMotor rightMotor({.in1 = BoardPins::RIGHT_MOTOR_IN1,
                             .in2 = BoardPins::RIGHT_MOTOR_IN2,
                             .pwm_hz = 20000,
                             .pwm_resolution_bits = 10});

    // ==========================
    // Sensors
    // ==========================

    ObstacleSensor irLeft({.pin = BoardPins::IR_LEFT, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false});
    ObstacleSensor irMid({.pin = BoardPins::IR_MIDDLE, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false});
    ObstacleSensor irRight({.pin = BoardPins::IR_RIGHT, .active_low = BoardPins::IR_ACTIVE_LOW, .debounce_ms = 30, .use_internal_pullup = false});

    bool irWatch = true;
    bool irPeriodic = false;

    Bh1750Sensor lightSensor({.wire = &Wire,
                              .address = 0x23,
                              .sample_period_ms = 250});

    bool luxPeriodic = true;

    I2sAudio audio({.bclk_pin = static_cast<int>(BoardPins::I2S_BCLK),
                    .lrclk_pin = static_cast<int>(BoardPins::I2S_LRCLK),
                    .dout_pin = static_cast<int>(BoardPins::I2S_DOUT),
                    .sample_rate_hz = 22050});

    // ==========================
    // Helpers
    // ==========================

    static inline float clampf(float v, float lo, float hi)
    {
        if (v < lo)
            return lo;
        if (v > hi)
            return hi;
        return v;
    }

    static inline bool approxEqual(float a, float b, float eps)
    {
        return std::fabs(a - b) <= eps;
    }

    static inline float slewTowards(float current, float target, float maxDelta)
    {
        const float delta = target - current;
        if (delta > maxDelta)
            return current + maxDelta;
        if (delta < -maxDelta)
            return current - maxDelta;
        return target;
    }

    static bool parseOnOffOr01(const String &s, bool &out)
    {
        String t = s;
        t.trim();
        t.toLowerCase();
        if (t == "on" || t == "true" || t == "1")
        {
            out = true;
            return true;
        }
        if (t == "off" || t == "false" || t == "0")
        {
            out = false;
            return true;
        }
        return false;
    }

    // ==========================
    // Drive control (product-grade)
    // ==========================

    struct DriveConfig
    {
        float throttle_deadband = 0.03f;
        float steer_deadband = 0.03f;

        float throttle_slew_rate = 2.8f;
        float steer_slew_rate = 4.5f;

        uint32_t manual_cmd_timeout_ms = 600;

        uint32_t motor_apply_min_interval_ms = 15;

        uint32_t drive_debug_interval_ms = 250;

        uint32_t obstacle_hold_ms = 300;

        bool renormalize_mixing = true;

        bool drive_debug = false;
    };

    DriveConfig driveCfg{};

    float targetThrottle = 0.0f;
    float targetSteer = 0.0f;

    float smoothedThrottle = 0.0f;
    float smoothedSteer = 0.0f;

    uint32_t lastDriveCmdMs = 0;
    uint32_t lastDriveUpdateMs = 0;

    uint32_t lastMotorApplyMs = 0;
    float lastAppliedLeft = 0.0f;
    float lastAppliedRight = 0.0f;

    uint32_t lastDriveDebugMs = 0;

    bool obstacleFrontActive = false;
    uint32_t obstacleHoldUntilMs = 0;

    static inline bool frontObstacleNow()
    {
        return irLeft.isObstacle() || irMid.isObstacle() || irRight.isObstacle();
    }

    void setDriveTargets(float throttle, float steer, bool immediate);

    void applyTank(float throttle, float steer, uint32_t nowMs)
    {
        throttle = clampf(throttle, -1.0f, 1.0f);
        steer = clampf(steer, -1.0f, 1.0f);

        if (std::fabs(throttle) < driveCfg.throttle_deadband)
            throttle = 0.0f;
        if (std::fabs(steer) < driveCfg.steer_deadband)
            steer = 0.0f;

        float left = throttle - steer;
        float right = throttle + steer;

        if (driveCfg.renormalize_mixing)
        {
            const float m = std::max(std::fabs(left), std::fabs(right));
            if (m > 1.0f)
            {
                left /= m;
                right /= m;
            }
        }

        left = clampf(left, -1.0f, 1.0f);
        right = clampf(right, -1.0f, 1.0f);

        constexpr float APPLY_EPS = 0.005f; // 0.5% changes
        if ((nowMs - lastMotorApplyMs) < driveCfg.motor_apply_min_interval_ms &&
            approxEqual(left, lastAppliedLeft, APPLY_EPS) &&
            approxEqual(right, lastAppliedRight, APPLY_EPS))
        {
            return;
        }

        if (approxEqual(left, lastAppliedLeft, APPLY_EPS) &&
            approxEqual(right, lastAppliedRight, APPLY_EPS))
        {
            return;
        }

        leftMotor.set(left);
        rightMotor.set(right);

        lastAppliedLeft = left;
        lastAppliedRight = right;
        lastMotorApplyMs = nowMs;

        if (driveCfg.drive_debug && (nowMs - lastDriveDebugMs) >= driveCfg.drive_debug_interval_ms)
        {
            lastDriveDebugMs = nowMs;
            Serial.printf("[drive] tgt(t=%.2f s=%.2f) sm(t=%.2f s=%.2f) -> L=%.2f R=%.2f obs=%d\n",
                          static_cast<double>(targetThrottle),
                          static_cast<double>(targetSteer),
                          static_cast<double>(smoothedThrottle),
                          static_cast<double>(smoothedSteer),
                          static_cast<double>(left),
                          static_cast<double>(right),
                          obstacleFrontActive ? 1 : 0);
        }
    }

    void setDriveTargets(float throttle, float steer, bool immediate)
    {
        targetThrottle = clampf(throttle, -1.0f, 1.0f);
        targetSteer = clampf(steer, -1.0f, 1.0f);
        lastDriveCmdMs = millis();

        if (immediate)
        {
            smoothedThrottle = targetThrottle;
            smoothedSteer = targetSteer;
            applyTank(smoothedThrottle, smoothedSteer, lastDriveCmdMs);
        }
    }

    void updateDrive(uint32_t nowMs)
    {
        if (lastDriveUpdateMs == 0)
            lastDriveUpdateMs = nowMs;

        float dt = static_cast<float>(nowMs - lastDriveUpdateMs) * 0.001f;
        lastDriveUpdateMs = nowMs;

        dt = clampf(dt, 0.0f, 0.100f);

        const bool obsNow = frontObstacleNow();
        if (obsNow)
        {
            obstacleHoldUntilMs = nowMs + driveCfg.obstacle_hold_ms;
            if (!obstacleFrontActive)
                Serial.println("[ir] obstacle FRONT detected -> forward blocked");
            obstacleFrontActive = true;
        }
        else
        {
            if (obstacleFrontActive && nowMs >= obstacleHoldUntilMs)
            {
                obstacleFrontActive = false;
                Serial.println("[ir] obstacle FRONT cleared -> forward allowed");
            }
        }

        if (driveMode == RobotHttpServer::DriveMode::MANUAL &&
            (nowMs - lastDriveCmdMs) > driveCfg.manual_cmd_timeout_ms)
        {
            targetThrottle = 0.0f;
            targetSteer = 0.0f;
        }

        if (obstacleFrontActive)
        {
            if (targetThrottle > 0.0f)
                targetThrottle = 0.0f;

            if (smoothedThrottle > 0.0f)
                smoothedThrottle = 0.0f;
        }

        const float maxThrottleStep = driveCfg.throttle_slew_rate * dt;
        const float maxSteerStep = driveCfg.steer_slew_rate * dt;

        smoothedThrottle = slewTowards(smoothedThrottle, targetThrottle, maxThrottleStep);
        smoothedSteer = slewTowards(smoothedSteer, targetSteer, maxSteerStep);

        if (obstacleFrontActive && smoothedThrottle > 0.0f)
            smoothedThrottle = 0.0f;

        applyTank(smoothedThrottle, smoothedSteer, nowMs);
    }

    // ==========================
    // Lux / IR prints
    // ==========================

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

    // ==========================
    // LED strip
    // ==========================

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

    // ==========================
    // OLED
    // ==========================

    OledDisplay oled({.wire = &Wire, .address = 0x3C, .width = 128, .height = 64, .reset_pin = -1});
    uint32_t lastOledMs = 0;

    void oledBootScreen()
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

    void oledUpdateTask(uint32_t nowMs)
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

        if (lightSensor.hasReading())
            l[2] = "Lux: " + String(lightSensor.lux(), 1);
        else
            l[2] = "Lux: n/a";

        l[3] = String("LED: ") + (ledEnabled ? "on" : "off") + (ledAutoEnabled ? " (A)" : " (M)");
        l[4] = String("WS: ") + (WsControlClient::isConnected() ? "connected" : "offline");

        if (obstacleFrontActive)
            l[5] = "IR: FRONT BLOCK";
        else
            l[5] = "";

        oled.setLines(l, 6);
        oled.show();
    }

    // ==========================
    // Console help / handling
    // ==========================

    void printHelp()
    {
        Serial.println("Commands:");
        Serial.println("  help                      - show commands");
        Serial.println("  l <v>                     - set left motor [-1.0..1.0] (direct, no smoothing)");
        Serial.println("  r <v>                     - set right motor [-1.0..1.0] (direct, no smoothing)");
        Serial.println("  tank <throttle> <steer>   - set targets [-1.0..1.0] (smoothed)");
        Serial.println("  stop                      - stop both motors");
        Serial.println("  drivedbg on|off            - drive debug prints");
        Serial.println("  ir                         - print IR sensor states once");
        Serial.println("  irwatch on|off              - print IR edge events");
        Serial.println("  irperiodic on|off           - periodic IR snapshot every 500ms");
        Serial.println("  lux                        - print light sensor lux once");
        Serial.println("  luxperiodic on|off          - periodic lux print every 500ms");
        Serial.println("  led on|off                 - manual LED on/off");
        Serial.println("  ledauto on|off             - enable/disable auto LED control");
        Serial.println("  ledcolor <r> <g> <b>       - set LED color (0..255)");
        Serial.println("  ledbri <x>                 - set LED brightness (0..255)");
        Serial.println("  vol <0..100>               - set audio volume percent");
        Serial.println("  beep                       - play short test beep");
        Serial.println("  beep <hz> <ms>             - play beep with frequency and duration");
    }

    void handleConsole()
    {
        String line;
        if (!console.pollLine(line))
            return;

        String trimmed = line;
        trimmed.trim();

        if (trimmed.equalsIgnoreCase("help"))
        {
            printHelp();
            return;
        }

        float a = 0.0f;
        float b = 0.0f;
        int ia = 0, ib = 0, ic = 0;

        if (sscanf(trimmed.c_str(), "l %f", &a) == 1)
        {
            leftMotor.set(clampf(a, -1.0f, 1.0f));
            return;
        }
        if (sscanf(trimmed.c_str(), "r %f", &a) == 1)
        {
            rightMotor.set(clampf(a, -1.0f, 1.0f));
            return;
        }

        if (sscanf(trimmed.c_str(), "tank %f %f", &a, &b) == 2)
        {
            setDriveTargets(a, b, false);
            return;
        }

        if (trimmed.equalsIgnoreCase("stop"))
        {
            setDriveTargets(0.0f, 0.0f, true);
            return;
        }

        if (trimmed.startsWith("drivedbg"))
        {
            int sp = trimmed.indexOf(' ');
            if (sp > 0)
            {
                bool on = false;
                if (parseOnOffOr01(trimmed.substring(sp + 1), on))
                {
                    driveCfg.drive_debug = on;
                    Serial.printf("[drive] debug %s\n", on ? "on" : "off");
                    return;
                }
            }
            Serial.println("[drive] usage: drivedbg on|off");
            return;
        }

        if (trimmed.equalsIgnoreCase("ir"))
        {
            printIrOnce();
            return;
        }

        if (trimmed.startsWith("irwatch"))
        {
            int sp = trimmed.indexOf(' ');
            if (sp > 0)
            {
                bool on = false;
                if (parseOnOffOr01(trimmed.substring(sp + 1), on))
                {
                    irWatch = on;
                    Serial.printf("[ir] watch %s\n", irWatch ? "on" : "off");
                    return;
                }
            }
            Serial.println("[ir] usage: irwatch on|off");
            return;
        }

        if (trimmed.startsWith("irperiodic"))
        {
            int sp = trimmed.indexOf(' ');
            if (sp > 0)
            {
                bool on = false;
                if (parseOnOffOr01(trimmed.substring(sp + 1), on))
                {
                    irPeriodic = on;
                    Serial.printf("[ir] periodic %s\n", irPeriodic ? "on" : "off");
                    return;
                }
            }
            Serial.println("[ir] usage: irperiodic on|off");
            return;
        }

        if (trimmed.equalsIgnoreCase("lux"))
        {
            printLuxOnce();
            return;
        }

        if (trimmed.startsWith("luxperiodic"))
        {
            int sp = trimmed.indexOf(' ');
            if (sp > 0)
            {
                bool on = false;
                if (parseOnOffOr01(trimmed.substring(sp + 1), on))
                {
                    luxPeriodic = on;
                    Serial.printf("[lux] periodic %s\n", luxPeriodic ? "on" : "off");
                    return;
                }
            }
            Serial.println("[lux] usage: luxperiodic on|off");
            return;
        }

        if (trimmed.startsWith("ledauto"))
        {
            int sp = trimmed.indexOf(' ');
            if (sp > 0)
            {
                bool on = false;
                if (parseOnOffOr01(trimmed.substring(sp + 1), on))
                {
                    ledAutoEnabled = on;
                    Serial.printf("[led] auto %s\n", ledAutoEnabled ? "on" : "off");
                    return;
                }
            }
            Serial.println("[led] usage: ledauto on|off");
            return;
        }

        if (trimmed.startsWith("led "))
        {
            int sp = trimmed.indexOf(' ');
            bool on = false;
            if (sp > 0 && parseOnOffOr01(trimmed.substring(sp + 1), on))
            {
                ledAutoEnabled = false;
                ledSetEnabled(on);
                return;
            }
        }

        if (sscanf(trimmed.c_str(), "ledcolor %d %d %d", &ia, &ib, &ic) == 3)
        {
            ledAutoEnabled = false;
            ledR = static_cast<uint8_t>(clampf(ia, 0, 255));
            ledG = static_cast<uint8_t>(clampf(ib, 0, 255));
            ledB = static_cast<uint8_t>(clampf(ic, 0, 255));
            ledSetEnabled(true);
            ledApply();
            return;
        }

        if (sscanf(trimmed.c_str(), "ledbri %d", &ia) == 1)
        {
            ledBrightness = static_cast<uint8_t>(clampf(ia, 0, 255));
            ledApply();
            return;
        }

        if (sscanf(trimmed.c_str(), "vol %d", &ia) == 1)
        {
            ia = static_cast<int>(clampf(ia, 0, 100));
            audio.setVolume(static_cast<float>(ia) / 100.0f);
            Serial.printf("[audio] volume=%d%%\n", ia);
            return;
        }

        if (trimmed.equalsIgnoreCase("beep"))
        {
            audio.playBeep(880, 150);
            return;
        }

        if (sscanf(trimmed.c_str(), "beep %d %d", &ia, &ib) == 2)
        {
            ia = static_cast<int>(clampf(ia, 20, 20000));
            ib = static_cast<int>(clampf(ib, 10, 5000));
            audio.playBeep(static_cast<uint16_t>(ia), static_cast<uint16_t>(ib));
            return;
        }

        Serial.printf("[console] unknown: %s\n", trimmed.c_str());
    }

    // ==========================
    // Periodic tasks
    // ==========================

    void statusPrintTask(uint32_t nowMs)
    {
        if (nowMs - lastStatusPrintMs < 2000)
            return;

        lastStatusPrintMs = nowMs;

        Serial.printf("[status] wifi=%s ip=%s ws=%s mode=%s obs_front=%d\n",
                      WifiManager::isConnected() ? "ok" : "no",
                      WifiManager::ip().c_str(),
                      WsControlClient::isConnected() ? "ok" : "no",
                      (driveMode == RobotHttpServer::DriveMode::IDLE) ? "IDLE" : (driveMode == RobotHttpServer::DriveMode::MANUAL) ? "MANUAL"
                                                                                                                                   : "AUTO",
                      obstacleFrontActive ? 1 : 0);
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

    // ==========================
    // Backend state
    // ==========================

    String lastRouteStart;
    String lastRouteEnd;
    String positionStr = "Home";

    static const char *driveModeToBackend()
    {
        if (driveMode == RobotHttpServer::DriveMode::MANUAL)
            return "MANUAL";
        if (driveMode == RobotHttpServer::DriveMode::AUTO)
            return (positionStr == "MOVING") ? "NAVIGATING" : "IDLE";
        return "IDLE";
    }

    void backendRegisterTask(uint32_t nowMs)
    {
        if (!WifiManager::isConnected())
            return;

        if (nowMs - lastBackendRegisterMs < 30000)
            return;
        lastBackendRegisterMs = nowMs;

        const bool ok = BackendClient::registerRobot(BackendConfig::ROBOT_PORT);
        Serial.printf("[backend] register %s\n", ok ? "ok" : "fail");
    }

    void backendStatePush()
    {
        if (!WifiManager::isConnected())
            return;

        BackendClient::postState(
            "OK",
            85,
            driveModeToBackend(),
            "EMPTY",
            positionStr.length() ? positionStr : String(""),
            lastRouteStart.length() ? lastRouteStart : String(""),
            lastRouteEnd.length() ? lastRouteEnd : String(""));
    }

    void backendStateTask(uint32_t nowMs)
    {
        if (!WifiManager::isConnected())
            return;

        if (nowMs - lastBackendStateMs < 1000)
            return;
        lastBackendStateMs = nowMs;

        backendStatePush();
    }

} // namespace

namespace App
{
    void setup()
    {
        Serial.begin(AppConfig::SERIAL_BAUD);
        delay(50);

        const uint32_t bootMs = millis();
        lastDriveCmdMs = bootMs;
        lastDriveUpdateMs = bootMs;
        lastMotorApplyMs = bootMs;
        lastDriveDebugMs = bootMs;
        obstacleFrontActive = false;
        obstacleHoldUntilMs = 0;

        console.begin();

        leftMotor.begin();
        rightMotor.begin();

        irLeft.begin();
        irMid.begin();
        irRight.begin();

        Wire.begin(static_cast<int>(BoardPins::I2C_SDA), static_cast<int>(BoardPins::I2C_SCL));
        Wire.setClock(100000);
        delay(50);

        auto scanI2C = []()
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
        };
        scanI2C();

        const bool dok = oled.begin();
        Serial.printf("[oled] init %s (addr=0x%02X)\n", dok ? "ok" : "fail", 0x3C);
        if (dok)
            oledBootScreen();

        const bool lok = lightSensor.begin();
        Serial.printf("[bh1750] init %s (addr=0x%02X)\n", lok ? "ok" : "fail", 0x23);

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
            backendStatePush();
        }

        RobotHttpServer::begin(
            BackendConfig::ROBOT_PORT,
            []() -> RobotHttpServer::StatusSnapshot
            {
                RobotHttpServer::StatusSnapshot s{};
                s.systemHealth = "OK";
                s.batteryLevel = 85;
                s.driveMode = driveMode;
                s.cargoStatus = "EMPTY";

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
                    setDriveTargets(0.0f, 0.0f, true);
                    positionStr = (positionStr.length() ? positionStr : String("Home"));
                }

                Serial.printf("[mode] set to %s\n",
                              (driveMode == RobotHttpServer::DriveMode::IDLE) ? "IDLE" : (driveMode == RobotHttpServer::DriveMode::MANUAL) ? "MANUAL"
                                                                                                                                           : "AUTO");

                backendStatePush();
            },
            [](const String &startNode, const String &endNode)
            {
                lastRouteStart = startNode;
                lastRouteEnd = endNode;
                Serial.printf("[route] selected %s -> %s\n", startNode.c_str(), endNode.c_str());
                backendStatePush();
            });

        WsControlClient::begin(
            WsControlClient::Handlers{
                .onConnected = []()
                {
                    Serial.println("[ws] connected");
                    backendStatePush(); },
                .onDisconnected = []()
                { Serial.println("[ws] disconnected"); },
                .onNavigate = [](const String &startNode, const String &destNode)
                {
                    lastRouteStart = startNode;
                    lastRouteEnd = destNode;

                    driveMode = RobotHttpServer::DriveMode::AUTO;
                    positionStr = "MOVING";

                    Serial.printf("[ws] NAVIGATE %s -> %s\n", startNode.c_str(), destNode.c_str());
                    BackendClient::postEvent("START_BUTTON_PRESSED");
                    backendStatePush(); },
                .onDriveCommand = [](float linear, float angular)
                {
                    driveMode = RobotHttpServer::DriveMode::MANUAL;
                    positionStr = "MOVING";

                    if (!std::isfinite(linear))
                        linear = 0.0f;
                    if (!std::isfinite(angular))
                        angular = 0.0f;

                    linear = clampf(linear, -1.0f, 1.0f);
                    angular = clampf(angular, -1.0f, 1.0f);

                    setDriveTargets(linear, angular, false);

                    backendStatePush(); },
                .onStop = []()
                {
                    setDriveTargets(0.0f, 0.0f, true);
                    driveMode = RobotHttpServer::DriveMode::IDLE;
                    positionStr = (positionStr == "MOVING") ? String("Home") : positionStr;

                    Serial.println("[ws] STOP");
                    backendStatePush(); },
                .onSetMode = [](const String &mode)
                {
                    if (mode == "IDLE")
                    {
                        driveMode = RobotHttpServer::DriveMode::IDLE;
                        setDriveTargets(0.0f, 0.0f, true);
                    }
                    else if (mode == "MANUAL")
                    {
                        driveMode = RobotHttpServer::DriveMode::MANUAL;
                    }
                    else if (mode == "AUTO")
                    {
                        driveMode = RobotHttpServer::DriveMode::AUTO;
                    }

                    Serial.printf("[ws] SET_MODE %s\n", mode.c_str());
                    backendStatePush(); },
                .onUnknownCommand = [](const String &cmd, const JsonDocument &)
                { Serial.printf("[ws] unknown command: %s\n", cmd.c_str()); }});

        Serial.println("[boot] motors + IR(front) + BH1750 + WS2812B + I2S audio + OLED + backend ws/http");
        Serial.println("[boot] obstacle policy: blocks FORWARD, reverse allowed");
        printHelp();
    }

    void loop()
    {
        const uint32_t nowMs = millis();

        statusPrintTask(nowMs);

        irUpdateTask(nowMs);
        luxUpdateTask(nowMs);

        oledUpdateTask(nowMs);
        ledAutoTask();

        handleConsole();
        updateDrive(nowMs);

        RobotHttpServer::handle();
        WsControlClient::loop();

        backendRegisterTask(nowMs);
        backendStateTask(nowMs);

        delay(1);
    }
} // namespace App
