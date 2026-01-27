#pragma once
#include <Arduino.h>

namespace BackendConfig
{
    static const char *HOST = "172.20.10.3";
    constexpr uint16_t PORT = 3003;    // HTTP
    constexpr uint16_t WS_PORT = 3004; // WebSocket
    constexpr uint16_t ROBOT_PORT = 8080;
}
