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
            s.batteryLevel = sensors.batteryLevel();
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

            s.powerValid = sensors.hasPowerMonitor();
            s.batteryVoltage = s.powerValid ? sensors.batteryVoltage() : 0.0f;
            s.batteryCurrentA = s.powerValid ? sensors.batteryCurrentA() : 0.0f;
            s.batteryPowerW = s.powerValid ? sensors.batteryPowerW() : 0.0f;

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
                pushState();
            },
            .onDisconnected = []() {},
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
            .onLed = [this](bool enabled, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
            {
                const uint8_t scaledBrightness = static_cast<uint8_t>((static_cast<uint16_t>(brightness) * 255U) / 100U);

                leds.setAutoEnabled(false);
                leds.setColor(r, g, b);
                leds.setBrightness(scaledBrightness);
                leds.setEnabled(enabled);
                leds.apply();

                Serial.printf("[ws] LED enabled=%d rgb=(%u,%u,%u) brightness=%u\n",
                              enabled ? 1 : 0,
                              static_cast<unsigned>(r),
                              static_cast<unsigned>(g),
                              static_cast<unsigned>(b),
                              static_cast<unsigned>(scaledBrightness));
            },
            .onAudioBeep = [this](uint32_t hz, uint32_t ms)
            {
                hz = static_cast<uint32_t>(clampf(static_cast<float>(hz), 20.0f, 20000.0f));
                ms = static_cast<uint32_t>(clampf(static_cast<float>(ms), 10.0f, 5000.0f));

                audio.playBeep(static_cast<uint16_t>(hz), static_cast<uint16_t>(ms));
                Serial.printf("[ws] AUDIO_BEEP hz=%lu ms=%lu\n",
                              static_cast<unsigned long>(hz),
                              static_cast<unsigned long>(ms));
            },
            .onAudioVolume = [this](float value)
            {
                value = clampf(value, 0.0f, 1.0f);
                audio.setVolume(value);
                Serial.printf("[ws] AUDIO_VOLUME value=%.2f\n", static_cast<double>(value));
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

    if (lastBackendRegisterMs != 0 && (nowMs - lastBackendRegisterMs) < 30000)
        return;

    if (!BackendClient::queueRegisterRobot(BackendConfig::ROBOT_PORT))
        return;

    lastBackendRegisterMs = nowMs;
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

    if (!BackendClient::queueState(
            "OK",
            sensors.batteryLevel(),
            state.driveModeToBackend(),
            "EMPTY",
            state.position().length() ? state.position() : String(""),
            state.lastRouteStart().length() ? state.lastRouteStart() : String(""),
            state.lastRouteEnd().length() ? state.lastRouteEnd() : String("")))
    {
        return;
    }

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
    if (!BackendClient::queueEvent(eventToPost))
        return;

    eventPending = false;
    pendingEvent = "";
    lastBackendEventMs = nowMs;
}
