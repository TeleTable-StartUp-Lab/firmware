#include "net/backend_client.h"
#include "net/backend_config.h"
#include <WiFi.h>
#include <HTTPClient.h>

namespace BackendClient
{

    bool registerRobot(uint16_t robotPort)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("[backend] register failed: wifi not connected");
            return false;
        }

        HTTPClient http;

        String url = String("http://") + BackendConfig::HOST + ":" + String(BackendConfig::PORT) + "/table/register";
        http.begin(url);
        http.addHeader("Content-Type", "application/json");

        String payload = String("{\"port\":") + String(robotPort) + "}";

        const int code = http.POST(payload);
        String resp = http.getString();
        http.end();

        Serial.printf("[backend] POST /table/register code=%d resp=%s\n", code, resp.c_str());
        return code >= 200 && code < 300;
    }

}
