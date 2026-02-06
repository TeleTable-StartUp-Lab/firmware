#include "app/backend_coordinator.h"

#include "app/app_utils.h"
#include "app_config.h"
#include "net/backend_client.h"
#include "net/backend_config.h"
#include "net/robot_http_server.h"
#include "net/wifi_manager.h"
#include "net/ws_control_client.h"

#include <cmath>

BackendCoordinator::BackendCoordinator(RobotState &stateRef, DriveController &driveRef, SensorSuite &sensorsRef, LedController &ledsRef, I2sAudio &audioRef)
    : state(stateRef),
      drive(driveRef),
      sensors(sensorsRef),
      leds(ledsRef),
      audio(audioRef),
      lastBackendRegisterMs(0),
      lastBackendStateMs(0),
      lastBackendEventMs(0),
      stateDirty(false),
      stateUrgent(false),
      eventPending(false),
      pendingEvent("")
{
}

void BackendCoordinator::begin()
{
    RobotHttpServer::begin(
        BackendConfig::ROBOT_PORT,
        [this]() -> RobotHttpServer::StatusSnapshot
        {
            RobotHttpServer::StatusSnapshot s{};
            s.systemHealth = "OK";
            s.batteryLevel = 85;
            s.driveMode = state.driveMode();
            s.cargoStatus = "EMPTY";

            s.lastRouteStart = state.lastRouteStart().length() ? state.lastRouteStart().c_str() : nullptr;
            s.lastRouteEnd = state.lastRouteEnd().length() ? state.lastRouteEnd().c_str() : nullptr;
            s.position = state.position().length() ? state.position().c_str() : nullptr;

            s.irLeft = sensors.isIrLeftObstacle();
            s.irMid = sensors.isIrMidObstacle();
            s.irRight = sensors.isIrRightObstacle();

            s.luxValid = sensors.hasLux();
            s.lux = s.luxValid ? sensors.lux() : 0.0f;

            s.ledEnabled = leds.isEnabled();
            s.ledAutoEnabled = leds.isAutoEnabled();

            s.audioVolume = audio.volume();
            return s;
        },
        [this](RobotHttpServer::DriveMode m)
        {
            state.setDriveMode(m);

            if (m == RobotHttpServer::DriveMode::IDLE)
            {
                drive.setTargets(0.0f, 0.0f, true);
                state.ensureHomeIfEmpty();
            }

            Serial.printf("[mode] set to %s\n",
                          (m == RobotHttpServer::DriveMode::IDLE) ? "IDLE" : (m == RobotHttpServer::DriveMode::MANUAL) ? "MANUAL"
                                                                                                                         : "AUTO");

            pushState();
        },
        [this](const String &startNode, const String &endNode)
        {
            state.setRoute(startNode, endNode);
            Serial.printf("[route] selected %s -> %s\n", startNode.c_str(), endNode.c_str());
            pushState();
        });

    WsControlClient::begin(
        WsControlClient::Handlers{
            .onConnected = [this]()
            {
                Serial.println("[ws] connected");
                pushState();
            },
            .onDisconnected = []()
            { Serial.println("[ws] disconnected"); },
            .onNavigate = [this](const String &startNode, const String &destNode)
            {
                state.setRoute(startNode, destNode);

                state.setDriveMode(RobotHttpServer::DriveMode::AUTO);
                state.setPosition("MOVING");

                Serial.printf("[ws] NAVIGATE %s -> %s\n", startNode.c_str(), destNode.c_str());
                pendingEvent = "START_BUTTON_PRESSED";
                eventPending = true;
                pushState();
            },
            .onDriveCommand = [this](float linear, float angular)
            {
                state.setDriveMode(RobotHttpServer::DriveMode::MANUAL);
                state.setPosition("MOVING");

                if (!std::isfinite(linear))
                    linear = 0.0f;
                if (!std::isfinite(angular))
                    angular = 0.0f;

                linear = clampf(linear, -1.0f, 1.0f);
                angular = clampf(angular, -1.0f, 1.0f);

                drive.setTargets(linear, angular, false);

                pushState();
            },
            .onStop = [this]()
            {
                drive.setTargets(0.0f, 0.0f, true);
                state.setDriveMode(RobotHttpServer::DriveMode::IDLE);
                if (state.position() == "MOVING")
                    state.setPosition("Home");

                Serial.println("[ws] STOP");
                pushState();
            },
            .onSetMode = [this](const String &mode)
            {
                if (mode == "IDLE")
                {
                    state.setDriveMode(RobotHttpServer::DriveMode::IDLE);
                    drive.setTargets(0.0f, 0.0f, true);
                }
                else if (mode == "MANUAL")
                {
                    state.setDriveMode(RobotHttpServer::DriveMode::MANUAL);
                }
                else if (mode == "AUTO")
                {
                    state.setDriveMode(RobotHttpServer::DriveMode::AUTO);
                }

                Serial.printf("[ws] SET_MODE %s\n", mode.c_str());
                pushState();
            },
            .onUnknownCommand = [](const String &cmd, const JsonDocument &)
            { Serial.printf("[ws] unknown command: %s\n", cmd.c_str()); }});
}

void BackendCoordinator::handle()
{
    RobotHttpServer::handle();
    WsControlClient::loop();
}

void BackendCoordinator::registerTask(uint32_t nowMs)
{
    if (!WifiManager::isConnected())
        return;

    if (nowMs - lastBackendRegisterMs < 30000)
        return;
    lastBackendRegisterMs = nowMs;

    const bool ok = BackendClient::registerRobot(BackendConfig::ROBOT_PORT);
    Serial.printf("[backend] register %s\n", ok ? "ok" : "fail");
}

void BackendCoordinator::pushState()
{
    stateDirty = true;
    stateUrgent = true;
}

void BackendCoordinator::stateTask(uint32_t nowMs)
{
    if (!WifiManager::isConnected())
        return;

    const bool heartbeatDue = (nowMs - lastBackendStateMs) >= AppConfig::BACKEND_STATE_HEARTBEAT_MS;
    const bool minGapMet = (nowMs - lastBackendStateMs) >= AppConfig::BACKEND_STATE_MIN_GAP_MS;

    if (!stateDirty && !heartbeatDue)
        return;

    if (stateUrgent && !minGapMet)
        return;

    BackendClient::postState(
        "OK",
        85,
        state.driveModeToBackend(),
        "EMPTY",
        state.position().length() ? state.position() : String(""),
        state.lastRouteStart().length() ? state.lastRouteStart() : String(""),
        state.lastRouteEnd().length() ? state.lastRouteEnd() : String(""));

    lastBackendStateMs = nowMs;
    stateDirty = false;
    stateUrgent = false;
}

void BackendCoordinator::eventTask(uint32_t nowMs)
{
    if (!WifiManager::isConnected())
        return;
    if (!eventPending)
        return;
    if ((nowMs - lastBackendEventMs) < AppConfig::BACKEND_EVENT_MIN_GAP_MS)
        return;

    const String eventToPost = pendingEvent;
    eventPending = false;
    pendingEvent = "";

    BackendClient::postEvent(eventToPost);
    lastBackendEventMs = nowMs;
}
