#include "app/navigation_controller.h"
#include "app/app_utils.h"
#include "app/firmware_alert.h"

#include <cmath>
#include <cstdarg>

namespace {
constexpr float NAV_DRIVE_THROTTLE = 0.3f;
constexpr float NAV_TURN_STEER = 1.0f;
constexpr float NAV_STRAIGHT_YAW_TARGET_DPS = 0.0f;
constexpr float NAV_STRAIGHT_YAW_DEADBAND_DPS = 1.5f;
constexpr float NAV_STRAIGHT_YAW_KP = 0.015f;
constexpr float NAV_STRAIGHT_YAW_MAX_STEER = 0.18f;
// This passes through the drive straight-steer deadband/expo, so the actual
// motor delta is much smaller than the raw steer input.
constexpr float NAV_RIGHT_MOTOR_TRIM_STEER = 0.11f;
constexpr float TARGET_TURN_DEGREES = 90.0f;
constexpr uint32_t MAX_TURN_TIME_MS = 8000;
constexpr uint8_t NAV_GRAPH_NODE_COUNT = 15;
constexpr uint8_t NAV_GRAPH_EDGE_COUNT = 28;

// need this to signal where its home is
constexpr char HOME_NODE_ID[] = "apotheke";

// Node IDs
constexpr char APOTHEKE_NODE_ID[] = "apotheke";
constexpr char NODE_6_ID[] = "node6";
constexpr char RAUM_6_ID[] = "raum6";
constexpr char NODE_5_ID[] = "node5";
constexpr char NODE_7_ID[] = "node7";
constexpr char RAUM_5_ID[] = "raum5";
constexpr char NODE_4_ID[] = "node4";
constexpr char RAUM_3_ID[] = "raum3";
constexpr char NODE_3_ID[] = "node3";
constexpr char RAUM_4_ID[] = "raum4";
constexpr char NODE_2_ID[] = "node2";
constexpr char RAUM_2_ID[] = "raum2";
constexpr char NODE_1_ID[] = "node1";
constexpr char RAUM_1_ID[] = "raum1";
constexpr char MENSA_NODE_ID[] = "mensa";

// Labels
constexpr char APOTHEKE_NODE_LABEL[] = "Apotheke";
constexpr char NODE_6_LABEL[] = "Node 6";
constexpr char RAUM_6_LABEL[] = "Raum 6";
constexpr char NODE_5_LABEL[] = "Node 5";
constexpr char NODE_7_LABEL[] = "Node 7";
constexpr char RAUM_5_LABEL[] = "Raum 5";
constexpr char NODE_4_LABEL[] = "Node 4";
constexpr char RAUM_3_LABEL[] = "Raum 3";
constexpr char NODE_3_LABEL[] = "Node 3";
constexpr char RAUM_4_LABEL[] = "Raum 4";
constexpr char NODE_2_LABEL[] = "Node 2";
constexpr char RAUM_2_LABEL[] = "Raum 2";
constexpr char NODE_1_LABEL[] = "Node 1";
constexpr char RAUM_1_LABEL[] = "Raum 1";
constexpr char MENSA_NODE_LABEL[] = "Mensa";

// RFID UIDs
constexpr char APOTHEKE_NODE_RFID[] = "6C:C6:A4:EE";
constexpr char NODE_6_RFID[] = "BC:AA:A2:EE";
constexpr char RAUM_6_RFID[] = "6C:E8:A2:EE";
constexpr char NODE_5_RFID[] = "CC:62:9E:EE";
constexpr char NODE_7_RFID[] = "D1:08:97:F5";
constexpr char RAUM_5_RFID[] = "CC:B7:A0:EE";
constexpr char NODE_4_RFID[] = "BC:68:9E:EE";
constexpr char RAUM_3_RFID[] = "6C:B9:A0:EE";
constexpr char NODE_3_RFID[] = "7C:68:9E:EE";
constexpr char RAUM_4_RFID[] = "AC:72:A0:EE";
constexpr char NODE_2_RFID[] = "4C:62:9E:EE";
constexpr char RAUM_2_RFID[] = "2C:B9:A0:EE";
constexpr char NODE_1_RFID[] = "CC:11:9E:EE";
constexpr char RAUM_1_RFID[] = "5C:67:A0:EE";
constexpr char MENSA_NODE_RFID[] = "5C:B9:9B:EE";

constexpr NavigationController::GraphNode GRAPH_NODES[NAV_GRAPH_NODE_COUNT] = {
    // index 0 is home/start. Robot starts at Apotheke facing north.
    {APOTHEKE_NODE_ID, APOTHEKE_NODE_LABEL, APOTHEKE_NODE_RFID}, // 0
    {NODE_6_ID, NODE_6_LABEL, NODE_6_RFID},                      // 1
    {RAUM_6_ID, RAUM_6_LABEL, RAUM_6_RFID},                      // 2
    {NODE_5_ID, NODE_5_LABEL, NODE_5_RFID},                      // 3
    {NODE_7_ID, NODE_7_LABEL, NODE_7_RFID},                      // 4
    {RAUM_5_ID, RAUM_5_LABEL, RAUM_5_RFID},                      // 5
    {NODE_4_ID, NODE_4_LABEL, NODE_4_RFID},                      // 6
    {RAUM_3_ID, RAUM_3_LABEL, RAUM_3_RFID},                      // 7
    {NODE_3_ID, NODE_3_LABEL, NODE_3_RFID},                      // 8
    {RAUM_4_ID, RAUM_4_LABEL, RAUM_4_RFID},                      // 9
    {NODE_2_ID, NODE_2_LABEL, NODE_2_RFID},                      // 10
    {RAUM_2_ID, RAUM_2_LABEL, RAUM_2_RFID},                      // 11
    {NODE_1_ID, NODE_1_LABEL, NODE_1_RFID},                      // 12
    {RAUM_1_ID, RAUM_1_LABEL, RAUM_1_RFID},                      // 13
    {MENSA_NODE_ID, MENSA_NODE_LABEL, MENSA_NODE_RFID},          // 14
};

constexpr NavigationController::GraphEdge GRAPH_EDGES[NAV_GRAPH_EDGE_COUNT] = {
    // Apotheke ↔ Node 6
    {0, 1, 0},   // Apotheke → Node 6   north
    {1, 0, 180}, // Node 6 → Apotheke   south

    // Node 6 ↔ Raum 6
    {1, 2, 90},  // Node 6 → Raum 6     east
    {2, 1, 270}, // Raum 6 → Node 6     west

    // Node 6 ↔ Node 5
    {1, 3, 0},   // Node 6 → Node 5     north
    {3, 1, 180}, // Node 5 → Node 6     south

    // Node 5 ↔ Node 7
    {3, 4, 0},   // Node 5 → Node 7     north
    {4, 3, 180}, // Node 7 → Node 5     south

    // Node 7 ↔ Raum 5
    {4, 5, 90},  // Node 7 → Raum 5     east
    {5, 4, 270}, // Raum 5 → Node 7     west

    // Node 5 ↔ Node 4
    {3, 6, 270}, // Node 5 → Node 4     west
    {6, 3, 90},  // Node 4 → Node 5     east

    // Node 4 ↔ Raum 3
    {6, 7, 270}, // Node 4 → Raum 3     west
    {7, 6, 90},  // Raum 3 → Node 4     east

    // Node 4 ↔ Node 3
    {6, 8, 0},   // Node 4 → Node 3     north
    {8, 6, 180}, // Node 3 → Node 4     south

    // Node 3 ↔ Raum 4
    {8, 9, 90},  // Node 3 → Raum 4     east
    {9, 8, 270}, // Raum 4 → Node 3     west

    // Node 3 ↔ Node 2
    {8, 10, 270}, // Node 3 → Node 2     west
    {10, 8, 90},  // Node 2 → Node 3     east

    // Node 2 ↔ Raum 2
    {10, 11, 270}, // Node 2 → Raum 2    west
    {11, 10, 90},  // Raum 2 → Node 2    east

    // Node 2 ↔ Node 1
    {10, 12, 180}, // Node 2 → Node 1    south
    {12, 10, 0},   // Node 1 → Node 2    north

    // Node 1 ↔ Raum 1
    {12, 13, 270}, // Node 1 → Raum 1    west
    {13, 12, 90},  // Raum 1 → Node 1    east

    // Node 1 ↔ Mensa
    {12, 14, 180}, // Node 1 → Mensa     south
    {14, 12, 0},   // Mensa → Node 1     north
};

void logNavInfo(const char *format, ...) {
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    Serial.print("[nav] ");
    Serial.println(buf);
    FirmwareAlert::info(String(buf));
}

void logNavWarn(const char *format, ...) {
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    Serial.print("[nav] ");
    Serial.println(buf);
    FirmwareAlert::warn(String(buf));
}

void logNavError(const char *format, ...) {
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    Serial.print("[nav] ");
    Serial.println(buf);
    FirmwareAlert::error(String(buf));
}
} // namespace

NavigationController::NavigationController(RobotState &stateRef,
                                           DriveController &driveRef,
                                           SensorSuite &sensorsRef,
                                           LedController &ledsRef,
                                           I2sAudio &audioRef)
    : state(stateRef), drive(driveRef), sensors(sensorsRef), leds(ledsRef),
      audio(audioRef), currentNodeIndex(-1), targetNodeIndex(-1),
      navigationActive(false), motionPhase(MotionPhase::IDLE),
      plannedStepCount(0), currentStepIndex(0), lastTurnSampleMs(0),
      turnStartMs(0), accumulatedTurnDegrees(0.0f), drivingReverse(false),
      needsHomeReinitialization(false) {}

void NavigationController::begin() {
    setLocalizedNode(0);
    state.setTargetNode("");
    state.setNavigationStatus("IDLE");
    state.setPosition(GRAPH_NODES[0].id);
}

void NavigationController::update(uint32_t nowMs) {
    processRfid(nowMs);

    if (!navigationActive)
        return;

    // TODO: IR obstacle detection temporarily disabled
    if (false && sensors.frontObstacleNow()) {
        logNavWarn("obstacle detected during navigation");
        setError("obstacle detected");
        return;
    }

    if (motionPhase == MotionPhase::DRIVING) {
        applyStraightDriveYawHold();
        return;
    }

    if (motionPhase != MotionPhase::TURNING)
        return;

    if (!sensors.hasImu()) {
        logNavError("turn aborted: IMU reading unavailable");
        setError("imu unavailable");
        return;
    }

    if (lastTurnSampleMs == 0)
        lastTurnSampleMs = nowMs;

    const uint32_t deltaMs =
        (nowMs >= lastTurnSampleMs) ? (nowMs - lastTurnSampleMs) : 0;
    lastTurnSampleMs = nowMs;

    // Apply a deadband so gyro noise doesn't falsely accumulate rotation!
    const float absGyro = std::fabs(sensors.imu().gyro_z_dps);
    if (absGyro > 3.0f) {
        accumulatedTurnDegrees +=
            absGyro * (static_cast<float>(deltaMs) * 0.001f);
    }

    if (accumulatedTurnDegrees >= TARGET_TURN_DEGREES) {
        if (pendingTurnAction == NavigationAction::TURN_RIGHT) {
            currentHeadingDegrees += 90.0f;
        } else if (pendingTurnAction == NavigationAction::TURN_LEFT) {
            currentHeadingDegrees -= 90.0f;
        }
        while (currentHeadingDegrees < 0.0f)
            currentHeadingDegrees += 360.0f;
        while (currentHeadingDegrees >= 360.0f)
            currentHeadingDegrees -= 360.0f;

        logNavInfo("Target degrees reached (%d), starting forward drive. New "
                   "heading: %.0f",
                   (int)accumulatedTurnDegrees, currentHeadingDegrees);
        drive.setTargets(0.0f, 0.0f, true);
        startDriving(nowMs, false);
        return;
    }

    if (nowMs >= turnStartMs && (nowMs - turnStartMs) >= MAX_TURN_TIME_MS) {
        logNavError("turn aborted: timeout");
        setError("turn timeout");
    }
}

bool NavigationController::requestNavigation(const String &startNodeId,
                                             const String &targetNodeId,
                                             String *errorMessage) {
    auto rejectRequest = [&](const char *message,
                             bool critical = false) -> bool {
        if (errorMessage)
            *errorMessage = message ? message : "";

        if (!navigationActive) {
            state.setNavigationStatus("ERROR");
            notifyStateChanged();
        }

        if (message && message[0] != '\0') {
            if (critical)
                logNavError("navigation request rejected: %s", message);
            else
                logNavWarn("navigation request rejected: %s", message);
        }
        return false;
    };

    const int8_t requestedStartIndex = findNodeIndex(startNodeId);
    if (requestedStartIndex < 0)
        return rejectRequest("unknown start node");

    const int8_t requestedTargetIndex = findNodeIndex(targetNodeId);
    if (requestedTargetIndex < 0)
        return rejectRequest("unknown target node");

    if (needsHomeReinitialization)
        return rejectRequest("manual mode requires home reinitialization");

    if (currentNodeIndex < 0) {
        return rejectRequest(
            "current localization lost, place robot at Home facing Kitchen",
            true);
    }

    int8_t actualStart = currentNodeIndex;
    if (actualStart == findNodeIndex(HOME_NODE_ID) &&
        startNodeId == HOME_NODE_ID) {
        currentHeadingDegrees = 0.0f;
        logNavInfo("Reset current heading to 0 at Home node");
    }

    PlannedStep nextSteps[MAX_PATH_STEPS] = {};
    uint8_t nextStepCount = 0;

    if (actualStart != requestedStartIndex) {
        // Need to build a two-part path: current -> requestedStart ->
        // requestedTarget
        PlannedStep part1[MAX_PATH_STEPS] = {};
        uint8_t count1 = 0;
        if (!buildPath(actualStart, requestedStartIndex, part1, count1))
            return rejectRequest("no path found to intermediate start");

        // The second path needs to resume from the updated simulated heading
        // buildPath normally relies on `currentHeadingDegrees`. We must
        // temporarily update it, or just let it simulate naturally. Wait.
        // buildPath uses `currentHeadingDegrees` internally. We will
        // temporarily advance `currentHeadingDegrees` to match the end of
        // part 1.
        float backupHeading = currentHeadingDegrees;

        for (uint8_t i = 0; i < count1; ++i) {
            nextSteps[nextStepCount++] = part1[i];
            if (part1[i].action == NavigationAction::TURN_RIGHT)
                currentHeadingDegrees += 90.0f;
            if (part1[i].action == NavigationAction::TURN_LEFT)
                currentHeadingDegrees -= 90.0f;
            while (currentHeadingDegrees < 0.0f)
                currentHeadingDegrees += 360.0f;
            while (currentHeadingDegrees >= 360.0f)
                currentHeadingDegrees -= 360.0f;
        }

        PlannedStep part2[MAX_PATH_STEPS] = {};
        uint8_t count2 = 0;
        if (!buildPath(requestedStartIndex, requestedTargetIndex, part2,
                       count2)) {
            currentHeadingDegrees = backupHeading; // Restore
            return rejectRequest("no path found to target");
        }

        if (static_cast<uint16_t>(count1) + count2 > MAX_PATH_STEPS) {
            currentHeadingDegrees = backupHeading;
            return rejectRequest("path too long");
        }

        for (uint8_t i = 0; i < count2; ++i) {
            nextSteps[nextStepCount++] = part2[i];
        }

        currentHeadingDegrees = backupHeading; // Restore for actual execution
    } else {
        if (!buildPath(requestedStartIndex, requestedTargetIndex, nextSteps,
                       nextStepCount))
            return rejectRequest("no path found");
    }

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
    drivingReverse = false;

    state.setRoute(startNodeId, targetNodeId);
    state.setTargetNode(targetNodeId);
    state.setDriveMode(RobotHttpServer::DriveMode::AUTO);
    state.setNavigationStatus("PLANNING");
    notifyStateChanged();

    logNavInfo("route accepted %s -> %s (%u steps)", startNodeId.c_str(),
               targetNodeId.c_str(), static_cast<unsigned>(plannedStepCount));

    if (plannedStepCount == 0) {
        navigationActive = false;
        drivingReverse = false;
        state.setNavigationStatus("ARRIVED");
        notifyStateChanged();
        logNavInfo("already at target %s", targetNodeId.c_str());
        return true;
    }

    startStep(millis());
    return true;
}

void NavigationController::cancel(const char *navigationStatus,
                                  bool clearTargetNode) {
    navigationActive = false;
    motionPhase = MotionPhase::IDLE;
    plannedStepCount = 0;
    currentStepIndex = 0;
    targetNodeIndex = clearTargetNode ? -1 : targetNodeIndex;
    accumulatedTurnDegrees = 0.0f;
    lastTurnSampleMs = 0;
    turnStartMs = 0;
    drivingReverse = false;

    stopMotion();

    if (clearTargetNode)
        state.setTargetNode("");
    state.setNavigationStatus(navigationStatus ? navigationStatus : "IDLE");
    notifyStateChanged();
}

void NavigationController::loseLocalization() {
    if (needsHomeReinitialization && currentNodeIndex < 0)
        return;

    currentNodeIndex = -1;
    currentHeadingDegrees = 0.0f;
    state.setCurrentNode("");
    state.setPosition("");
    needsHomeReinitialization = true;
    logNavWarn("Lost localization due to MANUAL override");
    notifyStateChanged();
}

void NavigationController::setStateChangedCallback(
    StateChangedCallback callback) {
    stateChangedCallback = callback;
}

int8_t NavigationController::findNodeIndex(const String &nodeId) const {
    for (uint8_t i = 0; i < GRAPH_NODE_COUNT; ++i) {
        if (nodeId == GRAPH_NODES[i].id)
            return static_cast<int8_t>(i);
    }
    return -1;
}

int8_t NavigationController::findNodeIndexByRfid(const String &rfidUid) const {
    for (uint8_t i = 0; i < GRAPH_NODE_COUNT; ++i) {
        if (rfidUid == GRAPH_NODES[i].rfidUid)
            return static_cast<int8_t>(i);
    }
    return -1;
}

bool NavigationController::buildPath(int8_t startNodeIndex,
                                     int8_t targetNodeIndex,
                                     PlannedStep *outSteps,
                                     uint8_t &outStepCount) const {
    outStepCount = 0;

    if (startNodeIndex == targetNodeIndex)
        return true;

    bool visited[GRAPH_NODE_COUNT] = {};
    int8_t previousNode[GRAPH_NODE_COUNT] = {};
    int8_t previousEdge[GRAPH_NODE_COUNT] = {};
    uint8_t queue[GRAPH_NODE_COUNT] = {};

    for (uint8_t i = 0; i < GRAPH_NODE_COUNT; ++i) {
        previousNode[i] = -1;
        previousEdge[i] = -1;
    }

    uint8_t queueHead = 0;
    uint8_t queueTail = 0;

    queue[queueTail++] = static_cast<uint8_t>(startNodeIndex);
    visited[startNodeIndex] = true;

    while (queueHead < queueTail) {
        const uint8_t node = queue[queueHead++];
        if (node == static_cast<uint8_t>(targetNodeIndex))
            break;

        for (uint8_t edgeIndex = 0; edgeIndex < GRAPH_EDGE_COUNT; ++edgeIndex) {
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

    int8_t reverseEdges[MAX_PATH_STEPS] = {};
    uint8_t reverseEdgeCount = 0;
    int8_t walk = targetNodeIndex;

    while (walk != startNodeIndex && reverseEdgeCount < MAX_PATH_STEPS) {
        const int8_t edgeIndex = previousEdge[walk];
        if (edgeIndex < 0)
            return false;

        reverseEdges[reverseEdgeCount++] = edgeIndex;
        walk = previousNode[walk];
    }

    int16_t simHeading = static_cast<int16_t>(currentHeadingDegrees);

    for (uint8_t i = 0; i < reverseEdgeCount; ++i) {
        const auto &edge = GRAPH_EDGES[reverseEdges[reverseEdgeCount - 1U - i]];

        int16_t diff = (edge.mapHeading - simHeading) % 360;
        if (diff < -180)
            diff += 360;
        else if (diff > 180)
            diff -= 360;

        NavigationAction act = NavigationAction::STRAIGHT;
        if (diff == 0) {
            act = NavigationAction::STRAIGHT;
        } else if (diff == 180 || diff == -180) {
            act = NavigationAction::REVERSE;
        } else if (diff == 90 || diff == -270) {
            act = NavigationAction::TURN_RIGHT;
            simHeading = (simHeading + 90) % 360;
        } else if (diff == -90 || diff == 270) {
            act = NavigationAction::TURN_LEFT;
            simHeading = (simHeading - 90) % 360;
        } else {
            act = NavigationAction::TURN_RIGHT;
            simHeading = (simHeading + diff) % 360;
        }

        if (simHeading < 0)
            simHeading += 360;

        outSteps[i] = PlannedStep{edge.from, edge.to, act};
    }

    outStepCount = reverseEdgeCount;
    return true;
}

void NavigationController::processRfid(uint32_t nowMs) {
    if (!sensors.hasRfid())
        return;

    if (state.driveMode() == RobotHttpServer::DriveMode::MANUAL)
        return;

    const String &uid = sensors.rfid().uid_hex;
    if (uid.length() == 0 || uid == lastSeenRfid)
        return;

    lastSeenRfid = uid;

    const int8_t nodeIndex = findNodeIndexByRfid(uid);
    if (nodeIndex < 0) {
        logNavWarn("ignoring unknown RFID %s", uid.c_str());
        return;
    }

    const int8_t homeIndex = findNodeIndex(HOME_NODE_ID);
    if (needsHomeReinitialization && nodeIndex != homeIndex)
        return;

    audio.playBeep(1000, 300);

    setLocalizedNode(nodeIndex);
    if (nodeIndex == homeIndex)
        needsHomeReinitialization = false;
    logNavInfo("localized at node %s (%s)", GRAPH_NODES[nodeIndex].id,
               uid.c_str());

    if (!navigationActive || motionPhase != MotionPhase::DRIVING)
        return;

    const PlannedStep &step = plannedSteps[currentStepIndex];
    if (nodeIndex != static_cast<int8_t>(step.to))
        return;

    completeStep(nowMs);
}

void NavigationController::applyStraightDriveYawHold() {
    const float throttle =
        drivingReverse ? -NAV_DRIVE_THROTTLE : NAV_DRIVE_THROTTLE;
    const float rightMotorTrim =
        drivingReverse ? -NAV_RIGHT_MOTOR_TRIM_STEER
                       : NAV_RIGHT_MOTOR_TRIM_STEER;
    float correction = 0.0f;

    if (sensors.hasImu()) {
        const float yawError =
            NAV_STRAIGHT_YAW_TARGET_DPS - sensors.imu().gyro_z_dps;
        if (std::fabs(yawError) > NAV_STRAIGHT_YAW_DEADBAND_DPS) {
            correction =
                clampf(yawError * NAV_STRAIGHT_YAW_KP,
                       -NAV_STRAIGHT_YAW_MAX_STEER, NAV_STRAIGHT_YAW_MAX_STEER);
        }
    }

    drive.setTargets(
        throttle,
        clampf(correction + rightMotorTrim, -NAV_STRAIGHT_YAW_MAX_STEER,
               NAV_STRAIGHT_YAW_MAX_STEER),
        false);
}

void NavigationController::startStep(uint32_t nowMs) {
    if (currentStepIndex >= plannedStepCount) {
        navigationActive = false;
        motionPhase = MotionPhase::IDLE;
        drivingReverse = false;
        state.setNavigationStatus("ARRIVED");
        notifyStateChanged();
        return;
    }

    const PlannedStep &step = plannedSteps[currentStepIndex];
    logNavInfo("step %u/%u %s -> %s action=%u",
               static_cast<unsigned>(currentStepIndex + 1U),
               static_cast<unsigned>(plannedStepCount),
               GRAPH_NODES[step.from].id, GRAPH_NODES[step.to].id,
               static_cast<unsigned>(step.action));

    if (step.action == NavigationAction::STRAIGHT) {
        startDriving(nowMs, false);
        return;
    }

    if (step.action == NavigationAction::REVERSE) {
        startDriving(nowMs, true);
        return;
    }

    startTurning(step.action, nowMs);
}

void NavigationController::startDriving(uint32_t, bool reverse) {
    motionPhase = MotionPhase::DRIVING;
    drivingReverse = reverse;
    state.setNavigationStatus("DRIVING");
    drive.setTargets(reverse ? -NAV_DRIVE_THROTTLE : NAV_DRIVE_THROTTLE,
                     reverse ? -NAV_RIGHT_MOTOR_TRIM_STEER
                             : NAV_RIGHT_MOTOR_TRIM_STEER,
                     true);
    notifyStateChanged();
}

void NavigationController::startTurning(NavigationAction action,
                                        uint32_t nowMs) {
    motionPhase = MotionPhase::TURNING;
    accumulatedTurnDegrees = 0.0f;
    lastTurnSampleMs = nowMs;
    turnStartMs = nowMs;
    pendingTurnAction = action;
    drivingReverse = false;

    const float steer = (action == NavigationAction::TURN_RIGHT)
                            ? -NAV_TURN_STEER
                            : NAV_TURN_STEER;
    state.setNavigationStatus("TURNING");
    drive.setTargets(0.0f, steer, true);
    notifyStateChanged();
}

void NavigationController::completeStep(uint32_t nowMs) {
    stopMotion();
    ++currentStepIndex;

    if (currentStepIndex >= plannedStepCount) {
        navigationActive = false;
        motionPhase = MotionPhase::IDLE;
        drivingReverse = false;
        state.setNavigationStatus("ARRIVED");
        notifyStateChanged();
        playArrivalJingle();
        leds.startArrivalCelebration();
        logNavInfo("arrived at %s", state.targetNode().c_str());
        return;
    }

    startStep(nowMs);
}

void NavigationController::playArrivalJingle() {
    audio.playBeep(784, 120);
    delay(30);
    audio.playBeep(988, 120);
    delay(30);
    audio.playBeep(1319, 180);
}

void NavigationController::stopMotion() { drive.setTargets(0.0f, 0.0f, true); }

void NavigationController::setLocalizedNode(int8_t nodeIndex) {
    if (nodeIndex < 0 || nodeIndex >= static_cast<int8_t>(GRAPH_NODE_COUNT))
        return;

    currentNodeIndex = nodeIndex;
    state.setCurrentNode(GRAPH_NODES[nodeIndex].id);
    state.setPosition(GRAPH_NODES[nodeIndex].id);
    if (nodeIndex == findNodeIndex(HOME_NODE_ID) && needsHomeReinitialization)
        currentHeadingDegrees = 0.0f;
    notifyStateChanged();
}

void NavigationController::setError(const char *message) {
    stopMotion();
    navigationActive = false;
    motionPhase = MotionPhase::IDLE;
    plannedStepCount = 0;
    currentStepIndex = 0;
    lastTurnSampleMs = 0;
    turnStartMs = 0;
    accumulatedTurnDegrees = 0.0f;
    drivingReverse = false;

    state.setDriveMode(RobotHttpServer::DriveMode::IDLE);
    state.setNavigationStatus("ERROR");
    notifyStateChanged();

    if (message && message[0] != '\0')
        logNavError("error: %s", message);
}

void NavigationController::notifyStateChanged() const {
    if (stateChangedCallback)
        stateChangedCallback();
}
