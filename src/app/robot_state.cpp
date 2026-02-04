#include "app/robot_state.h"

RobotState::RobotState()
    : driveModeValue(RobotHttpServer::DriveMode::IDLE),
      positionValue("Home")
{
}

RobotHttpServer::DriveMode RobotState::driveMode() const
{
    return driveModeValue;
}

void RobotState::setDriveMode(RobotHttpServer::DriveMode mode)
{
    driveModeValue = mode;
}

const String &RobotState::lastRouteStart() const
{
    return lastRouteStartValue;
}

const String &RobotState::lastRouteEnd() const
{
    return lastRouteEndValue;
}

const String &RobotState::position() const
{
    return positionValue;
}

void RobotState::setRoute(const String &startNode, const String &endNode)
{
    lastRouteStartValue = startNode;
    lastRouteEndValue = endNode;
}

void RobotState::setPosition(const String &pos)
{
    positionValue = pos;
}

void RobotState::ensureHomeIfEmpty()
{
    if (positionValue.length() == 0)
        positionValue = "Home";
}

const char *RobotState::driveModeToBackend() const
{
    if (driveModeValue == RobotHttpServer::DriveMode::MANUAL)
        return "MANUAL";
    if (driveModeValue == RobotHttpServer::DriveMode::AUTO)
        return (positionValue == "MOVING") ? "NAVIGATING" : "IDLE";
    return "IDLE";
}
