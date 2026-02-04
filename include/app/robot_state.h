#pragma once

#include <Arduino.h>

#include "net/robot_http_server.h"

class RobotState
{
public:
    RobotState();

    RobotHttpServer::DriveMode driveMode() const;
    void setDriveMode(RobotHttpServer::DriveMode mode);

    const String &lastRouteStart() const;
    const String &lastRouteEnd() const;
    const String &position() const;

    void setRoute(const String &startNode, const String &endNode);
    void setPosition(const String &pos);

    void ensureHomeIfEmpty();

    const char *driveModeToBackend() const;

private:
    RobotHttpServer::DriveMode driveModeValue;
    String lastRouteStartValue;
    String lastRouteEndValue;
    String positionValue;
};
