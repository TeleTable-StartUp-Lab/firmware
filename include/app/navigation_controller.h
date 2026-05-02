#pragma once

#include <Arduino.h>
#include <functional>

#include "app/drive_controller.h"
#include "app/robot_state.h"
#include "app/sensor_suite.h"
#include "drivers/i2s_audio.h"

class NavigationController
{
public:
    enum class NavigationAction : uint8_t
    {
        STRAIGHT,
        TURN_RIGHT,
        TURN_LEFT,
        REVERSE
    };

    struct GraphNode
    {
        const char *id;
        const char *label;
        const char *rfidUid;
    };

    struct GraphEdge
    {
        uint8_t from;
        uint8_t to;
        int16_t mapHeading;
    };

    using StateChangedCallback = std::function<void()>;

    NavigationController(RobotState &state, DriveController &drive, SensorSuite &sensors, I2sAudio &audio);

    void begin();
    void update(uint32_t nowMs);

    bool requestNavigation(const String &startNodeId, const String &targetNodeId, String *errorMessage = nullptr);
    void cancel(const char *navigationStatus = "IDLE", bool clearTargetNode = true);
    void loseLocalization();

    void setStateChangedCallback(StateChangedCallback callback);

private:
    struct PlannedStep
    {
        uint8_t from;
        uint8_t to;
        NavigationAction action;
    };

    enum class MotionPhase : uint8_t
    {
        IDLE,
        TURNING,
        DRIVING
    };

    static constexpr uint8_t GRAPH_NODE_COUNT = 3;
    static constexpr uint8_t GRAPH_EDGE_COUNT = 4;
    static constexpr uint8_t MAX_PATH_STEPS = 8;

    int8_t findNodeIndex(const String &nodeId) const;
    int8_t findNodeIndexByRfid(const String &rfidUid) const;
    bool buildPath(int8_t startNodeIndex, int8_t targetNodeIndex, PlannedStep *outSteps, uint8_t &outStepCount) const;

    void processRfid(uint32_t nowMs);
    void startStep(uint32_t nowMs);
    void startDriving(uint32_t nowMs, bool reverse = false);
    void startTurning(NavigationAction action, uint32_t nowMs);
    void completeStep(uint32_t nowMs);
    void stopMotion();
    void setLocalizedNode(int8_t nodeIndex);
    void setError(const char *message);
    void notifyStateChanged() const;

    float currentHeadingDegrees = 0.0f; // Track heading (0 = Home->Kitchen)

    RobotState &state;
    DriveController &drive;
    SensorSuite &sensors;
    I2sAudio &audio;
    StateChangedCallback stateChangedCallback;

    int8_t currentNodeIndex;
    int8_t targetNodeIndex;

    bool navigationActive;
    MotionPhase motionPhase;

    PlannedStep plannedSteps[MAX_PATH_STEPS];
    uint8_t plannedStepCount;
    uint8_t currentStepIndex;

    uint32_t lastTurnSampleMs;
    uint32_t turnStartMs;
    float accumulatedTurnDegrees;
    NavigationAction pendingTurnAction;
    bool needsHomeReinitialization;

    String lastSeenRfid;
};
