#include "app/robot_state.h"

RobotState::RobotState()
    : driveModeValue(RobotHttpServer::DriveMode::IDLE),
      navigationStatusValue("IDLE")
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

const String &RobotState::currentNode() const
{
    return currentNodeValue;
}

const String &RobotState::targetNode() const
{
    return targetNodeValue;
}

const String &RobotState::navigationStatus() const
{
    return navigationStatusValue;
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

void RobotState::setCurrentNode(const String &nodeUuid)
{
    currentNodeValue = nodeUuid;
    if (nodeUuid.length())
        positionValue = nodeUuid;
}

void RobotState::setTargetNode(const String &nodeUuid)
{
    targetNodeValue = nodeUuid;
}

void RobotState::setNavigationStatus(const String &status)
{
    navigationStatusValue = status;
}

void RobotState::ensureHomeIfEmpty()
{
    if (positionValue.length() == 0)
        positionValue = currentNodeValue.length() ? currentNodeValue : String("UNKNOWN");
}

const char *RobotState::driveModeToBackend() const
{
    if (driveModeValue == RobotHttpServer::DriveMode::MANUAL)
        return "MANUAL";
    if (driveModeValue == RobotHttpServer::DriveMode::AUTO)
        return (navigationStatusValue == "PLANNING" ||
                navigationStatusValue == "TURNING" ||
                navigationStatusValue == "DRIVING")
                   ? "NAVIGATING"
                   : "IDLE";
    return "IDLE";
}
