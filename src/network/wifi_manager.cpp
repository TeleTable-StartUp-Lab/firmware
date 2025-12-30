#include "wifi_manager.h"
#include "../include/secrets.h"

void WifiManager::connect()
{
    Serial.println("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // Wait for connection (non-blocking would be better, but okay for setup)
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

bool WifiManager::isConnected()
{
    return WiFi.status() == WL_CONNECTED;
}

String WifiManager::getIP()
{
    return WiFi.localIP().toString();
}