#include "app/app.h"
#include "app_config.h"
#include "board_pins.h"
#include "app/serial_console.h"
#include "drivers/hbridge_motor.h"
#include "drivers/obstacle_sensor.h"

namespace
{
    uint32_t lastHeartbeatMs = 0;
    uint32_t lastStatusPrintMs = 0;
    uint32_t lastIrPrintMs = 0;
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

    ObstacleSensor irLeft({.pin = BoardPins::IR_LEFT,
                           .active_low = BoardPins::IR_ACTIVE_LOW,
                           .debounce_ms = 30,
                           .use_internal_pullup = false});

    ObstacleSensor irMid({.pin = BoardPins::IR_MIDDLE,
                          .active_low = BoardPins::IR_ACTIVE_LOW,
                          .debounce_ms = 30,
                          .use_internal_pullup = false});

    ObstacleSensor irRight({.pin = BoardPins::IR_RIGHT,
                            .active_low = BoardPins::IR_ACTIVE_LOW,
                            .debounce_ms = 30,
                            .use_internal_pullup = false});

    bool irWatch = true;
    bool irPeriodic = true;

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

        Serial.println("[boot] base scaffold + left/right motor + IR sensors");
        printHelp();
    }

    void loop()
    {
        const uint32_t nowMs = millis();

        heartbeatTask(nowMs);
        statusPrintTask(nowMs);
        irUpdateTask(nowMs);
        handleConsole();

        delay(1);
    }
}
