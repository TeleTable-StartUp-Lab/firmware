#pragma once
#include <Arduino.h>

namespace BackendClient
{
    bool registerRobot(uint16_t robotPort);

    bool postState(const String &systemHealth,
                   int batteryLevel,
                   const String &driveMode,
                   const String &cargoStatus,
                   const String &currentPosition,
                   const String &lastNode,
                   const String &targetNode);

    bool postEvent(const String &eventName);
}
