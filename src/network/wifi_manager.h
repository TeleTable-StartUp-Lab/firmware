#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>

class WifiManager
{
public:
    // Initializes WiFi connection using credentials from secrets.h
    void connect();

    // Checks if WiFi is still connected, handles reconnects
    bool isConnected();

    // Returns the current local IP address as a string
    String getIP();
};

#endif