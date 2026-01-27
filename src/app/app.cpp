#include "app/app.h"
#include "app_config.h"
#include "board_pins.h"

namespace
{
    uint32_t lastHeartbeatMs = 0;
    uint32_t lastStatusPrintMs = 0;
    bool ledState = false;

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

        Serial.println("[boot] firmware-teletable base scaffold");
    }

    void loop()
    {
        const uint32_t nowMs = millis();
        heartbeatTask(nowMs);
        statusPrintTask(nowMs);
        delay(1);
    }

}
