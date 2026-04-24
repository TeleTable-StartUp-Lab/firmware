#pragma once
#include <Arduino.h>

namespace BackendClient
{
    struct PingResult
    {
        bool wifiConnected;
        bool targetValid;
        bool sessionStarted;
        bool sessionFinished;
        uint32_t transmitted;
        uint32_t received;
        uint32_t durationMs;
        uint32_t minTimeMs;
        uint32_t maxTimeMs;
        uint32_t avgTimeMs;
    };

    void begin();

    PingResult ping();

    bool registerRobot(uint16_t robotPort);
    bool queueRegisterRobot(uint16_t robotPort);

    bool postState(const String &systemHealth,
                   int batteryLevel,
                   const String &driveMode,
                   const String &cargoStatus,
                   const String &currentPosition,
                   const String &lastNode,
                   const String &targetNode,
                   bool hasGyroscope,
                   float gyroXDps,
                   float gyroYDps,
                   float gyroZDps,
                   bool hasRfid,
                   const String &lastReadUuid,
                   bool hasLux,
                   float lux,
                   bool hasInfrared,
                   bool infraredFront,
                   bool infraredLeft,
                   bool infraredRight,
                   bool hasPower,
                   float voltageV,
                   float currentA,
                   float powerW);
    bool queueState(const String &systemHealth,
                    int batteryLevel,
                    const String &driveMode,
                    const String &cargoStatus,
                    const String &currentPosition,
                    const String &lastNode,
                    const String &targetNode,
                    bool hasGyroscope,
                    float gyroXDps,
                    float gyroYDps,
                    float gyroZDps,
                    bool hasRfid,
                    const String &lastReadUuid,
                    bool hasLux,
                    float lux,
                    bool hasInfrared,
                    bool infraredFront,
                    bool infraredLeft,
                    bool infraredRight,
                    bool hasPower,
                    float voltageV,
                    float currentA,
                    float powerW);

    bool postEvent(const String &eventName);
    bool queueEvent(const String &eventName);
}
