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

    uint32_t getLastDriveCmdMs() const { return lastDriveCmdMs; }

private:
    struct DriveConfig
    {
        // Slightly reduced deadband for more responsive throttle
        float throttle_deadband = 0.03f;
        float steer_deadband = 0.07f;
        float straight_throttle_min = 0.15f;
        float straight_steer_deadband = 0.12f;
        float throttle_expo = 0.45f;
        float steer_expo = 0.75f;
        // Increase max overall throttle for higher top speed
        float max_throttle = 0.85f;
        float max_turn_throttle = 0.40f;
        float max_steer = 0.45f;
        // Allow slightly larger throttle while turning in-place and smoother response
        float in_place_throttle_max = 0.06f;
        float in_place_turn_scale = 0.40f;
        float turn_speed_reduction = 0.90f;
        // Increase slew rates for quicker, yet smooth speed changes
        float throttle_slew_rate = 3.5f;
        float steer_slew_rate = 3.5f;
        float timeout_brake_slew_rate = 9.0f;
        uint32_t manual_cmd_timeout_ms = 350;
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
