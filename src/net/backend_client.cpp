#include "net/backend_client.h"
#include "net/backend_config.h"
#include "secrets.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

namespace
{
    bool postJson(const char *path, const String &jsonBody)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("[backend] request failed: wifi not connected");
            return false;
        }

        HTTPClient http;
        String url = String("http://") + BackendConfig::HOST + ":" + String(BackendConfig::PORT) + String(path);

        http.begin(url);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("X-Api-Key", Secrets::ROBOT_API_KEY);

        const int code = http.POST(jsonBody);
        String resp = http.getString();
        http.end();

        Serial.printf("[backend] POST %s code=%d resp=%s\n", path, code, resp.c_str());
        return code >= 200 && code < 300;
    }
}

namespace BackendClient
{
    bool registerRobot(uint16_t robotPort)
    {
        JsonDocument doc;
        doc["port"] = robotPort;
        String payload;
        serializeJson(doc, payload);
        return postJson("/table/register", payload);
    }

    bool postState(const String &systemHealth,
                   int batteryLevel,
                   const String &driveMode,
                   const String &cargoStatus,
                   const String &currentPosition,
                   const String &lastNode,
                   const String &targetNode)
    {
        JsonDocument doc;
        doc["systemHealth"] = systemHealth;
        doc["batteryLevel"] = batteryLevel;
        doc["driveMode"] = driveMode;
        doc["cargoStatus"] = cargoStatus;
        doc["currentPosition"] = currentPosition;
        doc["lastNode"] = lastNode;
        doc["targetNode"] = targetNode;

        String payload;
        serializeJson(doc, payload);
        return postJson("/table/state", payload);
    }

    bool postEvent(const String &eventName)
    {
        JsonDocument doc;
        doc["event"] = eventName;
        doc["timestamp"] = (uint32_t)millis();

        String payload;
        serializeJson(doc, payload);
        return postJson("/table/event", payload);
    }
}
