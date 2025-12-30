#include "wifi_manager.h"
#include "../include/secrets.h"
#include "utils/logger.h"

void WifiManager::connect()
{
    Logger::info("WIFI", "Connecting to WiFi network...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // Wait for connection (non-blocking would be better, but okay for setup)
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }

    Logger::info("WIFI", "WiFi connected successfully");

    // Logging the IP address
    String ipMsg = "IP address: " + getIP();
    Logger::info("WIFI", ipMsg.c_str());
}

bool WifiManager::isConnected()
{
    return WiFi.status() == WL_CONNECTED;
}

String WifiManager::getIP()
{
    return WiFi.localIP().toString();
}