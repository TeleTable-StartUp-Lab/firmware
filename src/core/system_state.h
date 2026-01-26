#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

enum DriveMode
{
    IDLE,
    MANUAL,
    AUTO
};

struct SystemState
{
    SystemState();

    float lux;
    bool obstacleLeft;
    bool obstacleRight;
    int batteryLevel;

    float linearVelocity;
    float angularVelocity;

    DriveMode driveMode;
    String systemHealth;
    String currentPosition;

    unsigned long lastStatusUpdate;

    const char *driveModeToCString() const;
    bool setDriveModeFromString(const String &mode);

    void lock();
    void unlock();

private:
    SemaphoreHandle_t _mtx;
};

#endif
