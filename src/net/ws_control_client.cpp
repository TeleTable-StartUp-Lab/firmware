#include "net/ws_control_client.h"

#include "net/backend_config.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WebSocketsClient.h>

namespace
{
    WebSocketsClient g_ws;
    WsControlClient::Handlers g_handlers{};
    bool g_connected = false;

    uint32_t g_lastReconnectMs = 0;
    constexpr uint32_t RECONNECT_INTERVAL_MS = 3000;

    void handleJsonMessage(const char *payload, size_t len)
    {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, payload, len);
        if (err)
        {
            Serial.printf("[ws] json parse error: %s\n", err.c_str());
            return;
        }

        const char *cmd = doc["command"] | "";
        if (!cmd || cmd[0] == '\0')
        {
            Serial.println("[ws] missing 'command'");
            return;
        }

        if (strcmp(cmd, "NAVIGATE") == 0)
        {
            const String start = String((const char *)(doc["start"] | ""));
            const String dest = String((const char *)(doc["destination"] | ""));

            if (g_handlers.onNavigate)
                g_handlers.onNavigate(start, dest);
            return;
        }

        if (strcmp(cmd, "DRIVE_COMMAND") == 0)
        {
            const float linear = doc["linear_velocity"] | 0.0f;
            const float angular = doc["angular_velocity"] | 0.0f;

            if (g_handlers.onDriveCommand)
                g_handlers.onDriveCommand(linear, angular);
            return;
        }

        if (strcmp(cmd, "STOP") == 0)
        {
            if (g_handlers.onStop)
                g_handlers.onStop();
            return;
        }

        if (strcmp(cmd, "SET_MODE") == 0)
        {
            const String mode = String((const char *)(doc["mode"] | ""));
            if (g_handlers.onSetMode)
                g_handlers.onSetMode(mode);
            return;
        }

        Serial.printf("[ws] unknown command: %s\n", cmd);
        if (g_handlers.onUnknownCommand)
            g_handlers.onUnknownCommand(String(cmd), doc);
    }

    void wsEvent(WStype_t type, uint8_t *payload, size_t length)
    {
        switch (type)
        {
        case WStype_CONNECTED:
            g_connected = true;
            Serial.println("[ws] connected");
            if (g_handlers.onConnected)
                g_handlers.onConnected();
            break;

        case WStype_DISCONNECTED:
            g_connected = false;
            Serial.println("[ws] disconnected");
            if (g_handlers.onDisconnected)
                g_handlers.onDisconnected();
            break;

        case WStype_TEXT:
            if (payload && length > 0)
            {
                Serial.printf("[ws] rx: %.*s\n", (int)length, (const char *)payload);
                handleJsonMessage((const char *)payload, length);
            }
            break;

        case WStype_ERROR:
            Serial.println("[ws] error");
            break;

        default:
            break;
        }
    }
}

namespace WsControlClient
{
    void begin(const Handlers &handlers)
    {
        g_handlers = handlers;

        g_ws.begin(BackendConfig::HOST, BackendConfig::PORT, BackendConfig::WS_PATH);
        g_ws.onEvent(wsEvent);
        g_ws.setReconnectInterval(RECONNECT_INTERVAL_MS);
        g_ws.enableHeartbeat(15000, 3000, 2);
    }

    void loop()
    {
        g_ws.loop();

        if (WiFi.status() != WL_CONNECTED)
            return;

        const uint32_t nowMs = millis();
        if (!g_connected && (nowMs - g_lastReconnectMs) >= RECONNECT_INTERVAL_MS)
        {
            g_lastReconnectMs = nowMs;
            g_ws.disconnect();
            g_ws.begin(BackendConfig::HOST, BackendConfig::PORT, BackendConfig::WS_PATH);
        }
    }

    bool isConnected()
    {
        return g_connected;
    }

    bool sendJson(const JsonDocument &doc)
    {
        if (!g_connected)
            return false;

        String out;
        serializeJson(doc, out);
        g_ws.sendTXT(out.c_str());
        return true;
    }

    bool sendText(const String &text)
    {
        if (!g_connected)
            return false;

        g_ws.sendTXT(text.c_str());
        return true;
    }

    void disconnect()
    {
        g_ws.disconnect();
        g_connected = false;
    }
}
