#include "net/wifi_manager.h"
#include <WiFi.h>

namespace WifiManager
{

    bool begin(const char *ssid, const char *pass, uint32_t timeout_ms)
    {
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);

        WiFi.begin(ssid, pass);

        const uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED)
        {
            if (millis() - start > timeout_ms)
                return false;
            delay(50);
        }
        return true;
    }

    bool isConnected()
    {
        return WiFi.status() == WL_CONNECTED;
    }

    String ip()
    {
        if (!isConnected())
            return String("0.0.0.0");
        return WiFi.localIP().toString();
    }

}
