#include "app/navigation_controller.h"

#include <cmath>

namespace
{
    constexpr uint8_t NAV_GRAPH_NODE_COUNT = 3;
    constexpr uint8_t NAV_GRAPH_EDGE_COUNT = 4;

    constexpr char HOME_NODE_ID[] = "home";
    constexpr char KITCHEN_NODE_ID[] = "kitchen";
    constexpr char OFFICE_NODE_ID[] = "office";

    constexpr char HOME_NODE_LABEL[] = "Home";
    constexpr char KITCHEN_NODE_LABEL[] = "Kitchen";
    constexpr char OFFICE_NODE_LABEL[] = "Office";

    constexpr char HOME_NODE_RFID[] = "81:C7:97:F5";
    constexpr char KITCHEN_NODE_RFID[] = "B1:4E:96:F5";
    constexpr char OFFICE_NODE_RFID[] = "C1:0C:97:F5";

    constexpr float NAV_DRIVE_THROTTLE = 0.35f;
    constexpr float NAV_TURN_STEER = 1.0f;
    constexpr float TARGET_TURN_DEGREES = 90.0f;
    constexpr uint32_t MAX_TURN_TIME_MS = 8000;

    constexpr NavigationController::GraphNode GRAPH_NODES[NAV_GRAPH_NODE_COUNT] = {
        {HOME_NODE_ID, HOME_NODE_LABEL, HOME_NODE_RFID},
        {KITCHEN_NODE_ID, KITCHEN_NODE_LABEL, KITCHEN_NODE_RFID},
        {OFFICE_NODE_ID, OFFICE_NODE_LABEL, OFFICE_NODE_RFID},
    };

    constexpr NavigationController::GraphEdge GRAPH_EDGES[NAV_GRAPH_EDGE_COUNT] = {
        {0, 1, NavigationController::NavigationAction::STRAIGHT},
        {1, 0, NavigationController::NavigationAction::STRAIGHT},
        {1, 2, NavigationController::NavigationAction::TURN_RIGHT},
        {2, 1, NavigationController::NavigationAction::TURN_LEFT},
    };
}

NavigationController::NavigationController(RobotState &stateRef, DriveController &driveRef, SensorSuite &sensorsRef)
    : state(stateRef),
      drive(driveRef),
      sensors(sensorsRef),
      currentNodeIndex(-1),
      targetNodeIndex(-1),
      navigationActive(false),
      motionPhase(MotionPhase::IDLE),
      plannedStepCount(0),
      currentStepIndex(0),
      lastTurnSampleMs(0),
      turnStartMs(0),
      accumulatedTurnDegrees(0.0f)
{
}

void NavigationController::begin()
{
    setLocalizedNode(0);
    state.setTargetNode("");
    state.setNavigationStatus("IDLE");
    state.setPosition(GRAPH_NODES[0].id);
}

void NavigationController::update(uint32_t nowMs)
{
    processRfid(nowMs);

    if (!navigationActive)
        return;

    if (sensors.frontObstacleNow())
    {
        Serial.println("[nav] obstacle detected during navigation");
        setError("obstacle detected");
        return;
    }

    if (motionPhase != MotionPhase::TURNING)
        return;

    if (!sensors.hasImu())
    {
        Serial.println("[nav] turn aborted: IMU reading unavailable");
        setError("imu unavailable");
        return;
    }

    if (lastTurnSampleMs == 0)
        lastTurnSampleMs = nowMs;

    const uint32_t deltaMs = nowMs - lastTurnSampleMs;
    lastTurnSampleMs = nowMs;

    accumulatedTurnDegrees += std::fabs(sensors.imu().gyro_z_dps) * (static_cast<float>(deltaMs) * 0.001f);

    if (accumulatedTurnDegrees >= TARGET_TURN_DEGREES)
    {
        drive.setTargets(0.0f, 0.0f, true);
        startDriving(nowMs);
        return;
    }

    if ((nowMs - turnStartMs) >= MAX_TURN_TIME_MS)
    {
        Serial.println("[nav] turn aborted: timeout");
        setError("turn timeout");
    }
}

bool NavigationController::requestNavigation(const String &startNodeId, const String &targetNodeId, String *errorMessage)
{
    auto rejectRequest = [&](const char *message) -> bool
    {
        if (errorMessage)
            *errorMessage = message ? message : "";

        if (!navigationActive)
        {
            state.setNavigationStatus("ERROR");
            notifyStateChanged();
        }
        return false;
    };

    const int8_t requestedStartIndex = findNodeIndex(startNodeId);
    if (requestedStartIndex < 0)
        return rejectRequest("unknown start node");

    const int8_t requestedTargetIndex = findNodeIndex(targetNodeId);
    if (requestedTargetIndex < 0)
        return rejectRequest("unknown target node");

    if (currentNodeIndex != requestedStartIndex)
        return rejectRequest("start node does not match current localization");

    PlannedStep nextSteps[MAX_PATH_STEPS] = {};
    uint8_t nextStepCount = 0;
    if (!buildPath(requestedStartIndex, requestedTargetIndex, nextSteps, nextStepCount))
        return rejectRequest("no path found");

    stopMotion();

    for (uint8_t i = 0; i < nextStepCount; ++i)
        plannedSteps[i] = nextSteps[i];

    plannedStepCount = nextStepCount;
    currentStepIndex = 0;
    targetNodeIndex = requestedTargetIndex;
    navigationActive = true;
    motionPhase = MotionPhase::IDLE;
    accumulatedTurnDegrees = 0.0f;
    lastTurnSampleMs = 0;
    turnStartMs = 0;

    state.setRoute(startNodeId, targetNodeId);
    state.setTargetNode(targetNodeId);
    state.setDriveMode(RobotHttpServer::DriveMode::AUTO);
    state.setNavigationStatus("PLANNING");
    notifyStateChanged();

    Serial.printf("[nav] route accepted %s -> %s (%u steps)\n",
                  startNodeId.c_str(),
                  targetNodeId.c_str(),
                  static_cast<unsigned>(plannedStepCount));

    if (plannedStepCount == 0)
    {
        navigationActive = false;
        state.setNavigationStatus("ARRIVED");
        notifyStateChanged();
        Serial.printf("[nav] already at target %s\n", targetNodeId.c_str());
        return true;
    }

    startStep(millis());
    return true;
}

void NavigationController::cancel(const char *navigationStatus, bool clearTargetNode)
{
    navigationActive = false;
    motionPhase = MotionPhase::IDLE;
    plannedStepCount = 0;
    currentStepIndex = 0;
    targetNodeIndex = clearTargetNode ? -1 : targetNodeIndex;
    accumulatedTurnDegrees = 0.0f;
    lastTurnSampleMs = 0;
    turnStartMs = 0;

    stopMotion();

    if (clearTargetNode)
        state.setTargetNode("");
    state.setNavigationStatus(navigationStatus ? navigationStatus : "IDLE");
    notifyStateChanged();
}

void NavigationController::setStateChangedCallback(StateChangedCallback callback)
{
    stateChangedCallback = callback;
}

int8_t NavigationController::findNodeIndex(const String &nodeId) const
{
    for (uint8_t i = 0; i < GRAPH_NODE_COUNT; ++i)
    {
        if (nodeId == GRAPH_NODES[i].id)
            return static_cast<int8_t>(i);
    }
    return -1;
}

int8_t NavigationController::findNodeIndexByRfid(const String &rfidUid) const
{
    for (uint8_t i = 0; i < GRAPH_NODE_COUNT; ++i)
    {
        if (rfidUid == GRAPH_NODES[i].rfidUid)
            return static_cast<int8_t>(i);
    }
    return -1;
}

bool NavigationController::buildPath(int8_t startNodeIndex, int8_t targetNodeIndex, PlannedStep *outSteps, uint8_t &outStepCount) const
{
    outStepCount = 0;

    if (startNodeIndex == targetNodeIndex)
        return true;

    bool visited[GRAPH_NODE_COUNT] = {};
    int8_t previousNode[GRAPH_NODE_COUNT] = {-1, -1, -1};
    int8_t previousEdge[GRAPH_NODE_COUNT] = {-1, -1, -1};
    uint8_t queue[GRAPH_NODE_COUNT] = {};

    uint8_t queueHead = 0;
    uint8_t queueTail = 0;

    queue[queueTail++] = static_cast<uint8_t>(startNodeIndex);
    visited[startNodeIndex] = true;

    while (queueHead < queueTail)
    {
        const uint8_t node = queue[queueHead++];
        if (node == static_cast<uint8_t>(targetNodeIndex))
            break;

        for (uint8_t edgeIndex = 0; edgeIndex < GRAPH_EDGE_COUNT; ++edgeIndex)
        {
            const auto &edge = GRAPH_EDGES[edgeIndex];
            if (edge.from != node)
                continue;
            if (visited[edge.to])
                continue;

            visited[edge.to] = true;
            previousNode[edge.to] = static_cast<int8_t>(node);
            previousEdge[edge.to] = static_cast<int8_t>(edgeIndex);
            queue[queueTail++] = edge.to;
        }
    }

    if (!visited[targetNodeIndex])
        return false;

    int8_t reverseEdges[MAX_PATH_STEPS] = {-1, -1, -1};
    uint8_t reverseEdgeCount = 0;
    int8_t walk = targetNodeIndex;

    while (walk != startNodeIndex && reverseEdgeCount < MAX_PATH_STEPS)
    {
        const int8_t edgeIndex = previousEdge[walk];
        if (edgeIndex < 0)
            return false;

        reverseEdges[reverseEdgeCount++] = edgeIndex;
        walk = previousNode[walk];
    }

    for (uint8_t i = 0; i < reverseEdgeCount; ++i)
    {
        const auto &edge = GRAPH_EDGES[reverseEdges[reverseEdgeCount - 1U - i]];
        outSteps[i] = PlannedStep{edge.from, edge.to, edge.action};
    }

    outStepCount = reverseEdgeCount;
    return true;
}

void NavigationController::processRfid(uint32_t nowMs)
{
    if (!sensors.hasRfid())
        return;

    const String &uid = sensors.rfid().uid_hex;
    if (uid.length() == 0 || uid == lastSeenRfid)
        return;

    lastSeenRfid = uid;

    const int8_t nodeIndex = findNodeIndexByRfid(uid);
    if (nodeIndex < 0)
    {
        Serial.printf("[nav] ignoring unknown RFID %s\n", uid.c_str());
        return;
    }

    setLocalizedNode(nodeIndex);
    Serial.printf("[nav] localized at node %s (%s)\n",
                  GRAPH_NODES[nodeIndex].id,
                  uid.c_str());

    if (!navigationActive || motionPhase != MotionPhase::DRIVING)
        return;

    const PlannedStep &step = plannedSteps[currentStepIndex];
    if (nodeIndex != static_cast<int8_t>(step.to))
        return;

    completeStep(nowMs);
}

void NavigationController::startStep(uint32_t nowMs)
{
    if (currentStepIndex >= plannedStepCount)
    {
        navigationActive = false;
        motionPhase = MotionPhase::IDLE;
        state.setNavigationStatus("ARRIVED");
        notifyStateChanged();
        return;
    }

    const PlannedStep &step = plannedSteps[currentStepIndex];
    Serial.printf("[nav] step %u/%u %s -> %s action=%u\n",
                  static_cast<unsigned>(currentStepIndex + 1U),
                  static_cast<unsigned>(plannedStepCount),
                  GRAPH_NODES[step.from].id,
                  GRAPH_NODES[step.to].id,
                  static_cast<unsigned>(step.action));

    if (step.action == NavigationAction::STRAIGHT)
    {
        startDriving(nowMs);
        return;
    }

    startTurning(step.action, nowMs);
}

void NavigationController::startDriving(uint32_t)
{
    motionPhase = MotionPhase::DRIVING;
    state.setNavigationStatus("DRIVING");
    drive.setTargets(NAV_DRIVE_THROTTLE, 0.0f, true);
    notifyStateChanged();
}

void NavigationController::startTurning(NavigationAction action, uint32_t nowMs)
{
    motionPhase = MotionPhase::TURNING;
    accumulatedTurnDegrees = 0.0f;
    lastTurnSampleMs = nowMs;
    turnStartMs = nowMs;

    const float steer = (action == NavigationAction::TURN_RIGHT) ? NAV_TURN_STEER : -NAV_TURN_STEER;
    state.setNavigationStatus("TURNING");
    drive.setTargets(0.0f, steer, true);
    notifyStateChanged();
}

void NavigationController::completeStep(uint32_t nowMs)
{
    stopMotion();
    ++currentStepIndex;

    if (currentStepIndex >= plannedStepCount)
    {
        navigationActive = false;
        motionPhase = MotionPhase::IDLE;
        state.setNavigationStatus("ARRIVED");
        notifyStateChanged();
        Serial.printf("[nav] arrived at %s\n", state.targetNode().c_str());
        return;
    }

    startStep(nowMs);
}

void NavigationController::stopMotion()
{
    drive.setTargets(0.0f, 0.0f, true);
}

void NavigationController::setLocalizedNode(int8_t nodeIndex)
{
    if (nodeIndex < 0 || nodeIndex >= static_cast<int8_t>(GRAPH_NODE_COUNT))
        return;

    currentNodeIndex = nodeIndex;
    state.setCurrentNode(GRAPH_NODES[nodeIndex].id);
    state.setPosition(GRAPH_NODES[nodeIndex].id);
    notifyStateChanged();
}

void NavigationController::setError(const char *message)
{
    stopMotion();
    navigationActive = false;
    motionPhase = MotionPhase::IDLE;
    plannedStepCount = 0;
    currentStepIndex = 0;
    lastTurnSampleMs = 0;
    turnStartMs = 0;
    accumulatedTurnDegrees = 0.0f;

    state.setDriveMode(RobotHttpServer::DriveMode::IDLE);
    state.setNavigationStatus("ERROR");
    notifyStateChanged();

    if (message && message[0] != '\0')
        Serial.printf("[nav] error: %s\n", message);
}

void NavigationController::notifyStateChanged() const
{
    if (stateChangedCallback)
        stateChangedCallback();
}
