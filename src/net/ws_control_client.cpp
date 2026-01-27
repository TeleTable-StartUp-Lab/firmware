#include "net/ws_control_client.h"

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

namespace WsControlClient
{

    static WebSocketsClient ws;
    static Handlers h;
    static bool connected = false;

    static void handleText(const char *payload, size_t length)
    {
        JsonDocument doc;
        const DeserializationError err = deserializeJson(doc, payload, length);
        if (err)
        {
            Serial.println("[ws] invalid json");
            return;
        }

        const String type = doc["type"] | "";
        if (type.length() == 0)
        {
            Serial.println("[ws] missing type");
            return;
        }

        if (type == "stop")
        {
            if (h.onStop)
                h.onStop();
            return;
        }

        if (type == "tank")
        {
            TankCmd cmd{};
            cmd.throttle = doc["throttle"] | 0.0f;
            cmd.steer = doc["steer"] | 0.0f;
            if (h.onTank)
                h.onTank(cmd);
            return;
        }

        if (type == "mode")
        {
            const String mode = doc["mode"] | "";
            if (h.onMode)
                h.onMode(mode);
            return;
        }

        if (type == "ledcolor")
        {
            int r = doc["r"] | 0;
            int g = doc["g"] | 0;
            int b = doc["b"] | 0;
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
            if (h.onLedColor)
                h.onLedColor((uint8_t)r, (uint8_t)g, (uint8_t)b);
            return;
        }

        if (type == "vol")
        {
            int p = doc["percent"] | 0;
            if (p < 0)
                p = 0;
            if (p > 100)
                p = 100;
            if (h.onVolume)
                h.onVolume((uint8_t)p);
            return;
        }

        if (type == "beep")
        {
            int hz = doc["hz"] | 880;
            int ms = doc["ms"] | 150;
            if (hz < 50)
                hz = 50;
            if (hz > 5000)
                hz = 5000;
            if (ms < 10)
                ms = 10;
            if (ms > 2000)
                ms = 2000;
            if (h.onBeep)
                h.onBeep((uint16_t)hz, (uint16_t)ms);
            return;
        }

        Serial.printf("[ws] unknown type=%s\n", type.c_str());
    }

    static void onEvent(WStype_t type, uint8_t *payload, size_t length)
    {
        switch (type)
        {
        case WStype_CONNECTED:
            connected = true;
            Serial.println("[ws] connected");
            ws.sendTXT("{\"type\":\"robot\",\"msg\":\"hello\"}");
            break;

        case WStype_DISCONNECTED:
            connected = false;
            Serial.println("[ws] disconnected");
            break;

        case WStype_TEXT:
            handleText((const char *)payload, length);
            break;

        default:
            break;
        }
    }

    void begin(const char *host, uint16_t port, const char *path, const Handlers &handlers)
    {
        h = handlers;

        ws.begin(host, port, path);
        ws.onEvent(onEvent);
        ws.setReconnectInterval(2000);
        //  ws.enableHeartbeat(15000, 3000, 2);

        Serial.printf("[ws] connecting to ws://%s:%u%s\n", host, (unsigned)port, path);
    }

    void loop()
    {
        if (WiFi.status() != WL_CONNECTED)
            return;
        ws.loop();
    }

    bool isConnected()
    {
        return connected;
    }

}
