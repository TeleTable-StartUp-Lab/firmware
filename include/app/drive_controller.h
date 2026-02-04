#pragma once

#include <Arduino.h>

#include "app/sensor_suite.h"
#include "drivers/hbridge_motor.h"
#include "net/robot_http_server.h"

class DriveController
{
public:
    explicit DriveController(SensorSuite &sensors);

    void begin(uint32_t bootMs);

    void setTargets(float throttle, float steer, bool immediate);
    void update(uint32_t nowMs, RobotHttpServer::DriveMode mode);

    void setDebug(bool on);
    bool debugEnabled() const;

    void setLeftDirect(float v);
    void setRightDirect(float v);

    bool obstacleFrontActive() const;

private:
    struct DriveConfig
    {
        float throttle_deadband = 0.03f;
        float steer_deadband = 0.03f;
        float throttle_slew_rate = 2.8f;
        float steer_slew_rate = 4.5f;
        uint32_t manual_cmd_timeout_ms = 600;
        uint32_t motor_apply_min_interval_ms = 15;
        uint32_t drive_debug_interval_ms = 250;
        uint32_t obstacle_hold_ms = 300;
        bool renormalize_mixing = true;
        bool drive_debug = false;
    };

    void applyTank(float throttle, float steer, uint32_t nowMs);

    SensorSuite &sensors;

    HBridgeMotor leftMotor;
    HBridgeMotor rightMotor;

    DriveConfig cfg;

    float targetThrottle;
    float targetSteer;
    float smoothedThrottle;
    float smoothedSteer;

    uint32_t lastDriveCmdMs;
    uint32_t lastDriveUpdateMs;

    uint32_t lastMotorApplyMs;
    float lastAppliedLeft;
    float lastAppliedRight;

    uint32_t lastDriveDebugMs;

    bool obstacleFront;
    uint32_t obstacleHoldUntilMs;
};
