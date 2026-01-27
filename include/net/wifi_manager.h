#pragma once
#include <Arduino.h>

namespace WifiManager
{
    bool begin(const char *ssid, const char *pass, uint32_t timeout_ms);
    bool isConnected();
    String ip();
}
