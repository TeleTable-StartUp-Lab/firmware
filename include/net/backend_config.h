#pragma once
#include <Arduino.h>

namespace BackendConfig
{
    static const char *HOST = "10.10.31.13";
    constexpr uint16_t PORT = 443; // HTTPS + WSS
    static const char *WS_PATH = "/ws/robot/control";
    static const char *PING_PATH = "/health";
    constexpr bool USE_TLS = true;
    constexpr bool TLS_INSECURE = true;
    static const char *TLS_CA_CERT = nullptr;
    constexpr uint16_t ROBOT_PORT = 8080;
}
