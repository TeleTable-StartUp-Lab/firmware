#pragma once
#include <Arduino.h>
#include <functional>

namespace RobotHttpServer
{

    enum class DriveMode : uint8_t
    {
        IDLE,
        MANUAL,
        AUTO
    };

    struct StatusSnapshot
    {
        const char *systemHealth; // e.g. "OK"
        int batteryLevel;         // 0..100
        DriveMode driveMode;      // IDLE/MANUAL/AUTO
        const char *cargoStatus;  // e.g. "IN_TRANSIT"

        const char *lastRouteStart; // nullable
        const char *lastRouteEnd;   // nullable
        const char *position;       // nullable

        bool irLeft;
        bool irMid;
        bool irRight;

        float lux;
        bool luxValid;

        bool ledEnabled;
        bool ledAutoEnabled;

        float audioVolume; // 0..1
    };

    using StatusProvider = std::function<StatusSnapshot()>;
    using ModeSetter = std::function<void(DriveMode)>;
    using RouteSetter = std::function<void(const String &startNode, const String &endNode)>;

    void begin(uint16_t port, StatusProvider statusProvider, ModeSetter modeSetter, RouteSetter routeSetter);
    void handle();

}
