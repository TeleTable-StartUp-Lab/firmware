#include "app/app.h"
#include "app_config.h"
#include "board_pins.h"
#include "app/serial_console.h"
#include "drivers/hbridge_motor.h"

namespace
{
    uint32_t lastHeartbeatMs = 0;
    uint32_t lastStatusPrintMs = 0;
    bool ledState = false;

    SerialConsole console;

    HBridgeMotor leftMotor({.in1 = BoardPins::LEFT_MOTOR_IN1,
                            .in2 = BoardPins::LEFT_MOTOR_IN2,
                            .pwm_hz = 20000,
                            .pwm_resolution_bits = 10});

    void printHelp()
    {
        Serial.println("Commands:");
        Serial.println("  help        - show commands");
        Serial.println("  m <v>       - set motor [-1.0..1.0] (forward/reverse)");
        Serial.println("  stop        - motor coast stop");
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
            Serial.println("[motor] stop");
            return;
        }

        if (line.startsWith("m "))
        {
            const float v = line.substring(2).toFloat();
            leftMotor.set(v);
            Serial.printf("[motor] set=%.3f\n", static_cast<double>(v));
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

        Serial.println("[boot] base scaffold + left motor test");
        printHelp();
    }

    void loop()
    {
        const uint32_t nowMs = millis();

        heartbeatTask(nowMs);
        statusPrintTask(nowMs);
        handleConsole();

        delay(1);
    }
}
