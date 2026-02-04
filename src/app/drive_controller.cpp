#include "app/drive_controller.h"

#include "app/app_utils.h"
#include "board_pins.h"

#include <algorithm>
#include <cmath>

DriveController::DriveController(SensorSuite &sensorsRef)
    : sensors(sensorsRef),
      leftMotor({.in1 = BoardPins::LEFT_MOTOR_IN1,
                 .in2 = BoardPins::LEFT_MOTOR_IN2,
                 .pwm_hz = 20000,
                 .pwm_resolution_bits = 10}),
      rightMotor({.in1 = BoardPins::RIGHT_MOTOR_IN1,
                  .in2 = BoardPins::RIGHT_MOTOR_IN2,
                  .pwm_hz = 20000,
                  .pwm_resolution_bits = 10}),
      targetThrottle(0.0f),
      targetSteer(0.0f),
      smoothedThrottle(0.0f),
      smoothedSteer(0.0f),
      lastDriveCmdMs(0),
      lastDriveUpdateMs(0),
      lastMotorApplyMs(0),
      lastAppliedLeft(0.0f),
      lastAppliedRight(0.0f),
      lastDriveDebugMs(0),
      obstacleFront(false),
      obstacleHoldUntilMs(0)
{
}

void DriveController::begin(uint32_t bootMs)
{
    lastDriveCmdMs = bootMs;
    lastDriveUpdateMs = bootMs;
    lastMotorApplyMs = bootMs;
    lastDriveDebugMs = bootMs;
    obstacleFront = false;
    obstacleHoldUntilMs = 0;

    leftMotor.begin();
    rightMotor.begin();
}

void DriveController::setTargets(float throttle, float steer, bool immediate)
{
    targetThrottle = clampf(throttle, -1.0f, 1.0f);
    targetSteer = clampf(steer, -1.0f, 1.0f);
    lastDriveCmdMs = millis();

    if (immediate)
    {
        smoothedThrottle = targetThrottle;
        smoothedSteer = targetSteer;
        applyTank(smoothedThrottle, smoothedSteer, lastDriveCmdMs);
    }
}

void DriveController::update(uint32_t nowMs, RobotHttpServer::DriveMode mode)
{
    if (lastDriveUpdateMs == 0)
        lastDriveUpdateMs = nowMs;

    float dt = static_cast<float>(nowMs - lastDriveUpdateMs) * 0.001f;
    lastDriveUpdateMs = nowMs;

    dt = clampf(dt, 0.0f, 0.100f);

    const bool obsNow = sensors.frontObstacleNow();
    if (obsNow)
    {
        obstacleHoldUntilMs = nowMs + cfg.obstacle_hold_ms;
        if (!obstacleFront)
            Serial.println("[ir] obstacle FRONT detected -> forward blocked");
        obstacleFront = true;
    }
    else
    {
        if (obstacleFront && nowMs >= obstacleHoldUntilMs)
        {
            obstacleFront = false;
            Serial.println("[ir] obstacle FRONT cleared -> forward allowed");
        }
    }

    if (mode == RobotHttpServer::DriveMode::MANUAL &&
        (nowMs - lastDriveCmdMs) > cfg.manual_cmd_timeout_ms)
    {
        targetThrottle = 0.0f;
        targetSteer = 0.0f;
    }

    if (obstacleFront)
    {
        if (targetThrottle > 0.0f)
            targetThrottle = 0.0f;

        if (smoothedThrottle > 0.0f)
            smoothedThrottle = 0.0f;
    }

    const float maxThrottleStep = cfg.throttle_slew_rate * dt;
    const float maxSteerStep = cfg.steer_slew_rate * dt;

    smoothedThrottle = slewTowards(smoothedThrottle, targetThrottle, maxThrottleStep);
    smoothedSteer = slewTowards(smoothedSteer, targetSteer, maxSteerStep);

    if (obstacleFront && smoothedThrottle > 0.0f)
        smoothedThrottle = 0.0f;

    applyTank(smoothedThrottle, smoothedSteer, nowMs);
}

void DriveController::setDebug(bool on)
{
    cfg.drive_debug = on;
}

bool DriveController::debugEnabled() const
{
    return cfg.drive_debug;
}

void DriveController::setLeftDirect(float v)
{
    leftMotor.set(clampf(v, -1.0f, 1.0f));
}

void DriveController::setRightDirect(float v)
{
    rightMotor.set(clampf(v, -1.0f, 1.0f));
}

bool DriveController::obstacleFrontActive() const
{
    return obstacleFront;
}

void DriveController::applyTank(float throttle, float steer, uint32_t nowMs)
{
    throttle = clampf(throttle, -1.0f, 1.0f);
    steer = clampf(steer, -1.0f, 1.0f);

    if (std::fabs(throttle) < cfg.throttle_deadband)
        throttle = 0.0f;
    if (std::fabs(steer) < cfg.steer_deadband)
        steer = 0.0f;

    float left = throttle - steer;
    float right = throttle + steer;

    if (cfg.renormalize_mixing)
    {
        const float m = std::max(std::fabs(left), std::fabs(right));
        if (m > 1.0f)
        {
            left /= m;
            right /= m;
        }
    }

    left = clampf(left, -1.0f, 1.0f);
    right = clampf(right, -1.0f, 1.0f);

    constexpr float APPLY_EPS = 0.005f;
    if ((nowMs - lastMotorApplyMs) < cfg.motor_apply_min_interval_ms &&
        approxEqual(left, lastAppliedLeft, APPLY_EPS) &&
        approxEqual(right, lastAppliedRight, APPLY_EPS))
    {
        return;
    }

    if (approxEqual(left, lastAppliedLeft, APPLY_EPS) &&
        approxEqual(right, lastAppliedRight, APPLY_EPS))
    {
        return;
    }

    leftMotor.set(left);
    rightMotor.set(right);

    lastAppliedLeft = left;
    lastAppliedRight = right;
    lastMotorApplyMs = nowMs;

    if (cfg.drive_debug && (nowMs - lastDriveDebugMs) >= cfg.drive_debug_interval_ms)
    {
        lastDriveDebugMs = nowMs;
        Serial.printf("[drive] tgt(t=%.2f s=%.2f) sm(t=%.2f s=%.2f) -> L=%.2f R=%.2f obs=%d\n",
                      static_cast<double>(targetThrottle),
                      static_cast<double>(targetSteer),
                      static_cast<double>(smoothedThrottle),
                      static_cast<double>(smoothedSteer),
                      static_cast<double>(left),
                      static_cast<double>(right),
                      obstacleFront ? 1 : 0);
    }
}
