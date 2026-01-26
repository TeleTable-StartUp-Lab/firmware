#include "network/backend_client.h"
#include <WiFi.h>
#include "utils/logger.h"

// TEMP backend config placeholder
static const char *BACKEND_HOST = "172.20.10.3";
static const uint16_t BACKEND_WS_PORT = 3003;

BackendClient::BackendClient(uint16_t udpPort, uint16_t robotHttpPort, SystemState *state)
    : _udpPort(udpPort), _robotHttpPort(robotHttpPort), _state(state) {}

void BackendClient::begin()
{
    _udp.begin(0);
    setupWebSocket();

    Logger::info("BACKEND", "BackendClient initialized (UDP + WS)");
    _lastAnnounceMs = 0;
}

void BackendClient::loop()
{
    if (WiFi.status() != WL_CONNECTED)
        return;

    const unsigned long now = millis();
    if (now - _lastAnnounceMs >= ANNOUNCE_INTERVAL_MS)
    {
        sendUdpAnnounce();
        _lastAnnounceMs = now;
    }

    _ws.loop();
}

void BackendClient::setupWebSocket()
{
    _ws.begin(BACKEND_HOST, BACKEND_WS_PORT, "/ws/robot/control");
    _ws.setReconnectInterval(5000);

    _ws.onEvent([this](WStype_t type, uint8_t *payload, size_t length)
                { this->onWebSocketEvent(type, payload, length); });

    Logger::info("BACKEND", "WebSocket client setup complete");
}

void BackendClient::onWebSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_CONNECTED:
        Logger::info("BACKEND", "WebSocket connected");
        break;

    case WStype_DISCONNECTED:
        Logger::warn("BACKEND", "WebSocket disconnected (will retry)");
        break;

    case WStype_TEXT:
    {
        String msg((char *)payload, length);
        handleWsText(msg);
        break;
    }

    default:
        break;
    }
}

void BackendClient::handleWsText(const String &msg)
{
    Logger::info("BACKEND", ("WS TEXT: " + msg).c_str());

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, msg);
    if (err)
    {
        Logger::error("BACKEND", "WS JSON parse failed");
        return;
    }

    const char *command = doc["command"] | "";

    if (strcmp(command, "SET_MODE") == 0)
    {
        if (!_state)
            return;

        String mode = doc["mode"] | "IDLE";

        bool ok = false;
        _state->lock();
        ok = _state->setDriveModeFromString(mode);
        _state->unlock();

        if (ok)
            Logger::info("BACKEND", ("Drive mode set to " + mode).c_str());
        else
            Logger::warn("BACKEND", ("Invalid mode string: " + mode).c_str());

        return;
    }

    if (strcmp(command, "DRIVE_COMMAND") == 0)
    {
        if (!_state)
            return;

        bool manual = false;
        _state->lock();
        manual = (_state->driveMode == MANUAL);
        _state->unlock();

        if (!manual)
        {
            Logger::warn("BACKEND", "DRIVE_COMMAND ignored (not in MANUAL mode)");
            return;
        }

        float linear = doc["linear_velocity"] | 0.0f;
        float angular = doc["angular_velocity"] | 0.0f;

        _state->lock();
        _state->linearVelocity = linear;
        _state->angularVelocity = angular;
        _state->unlock();

        Logger::info(
            "BACKEND",
            ("Manual drive cmd: linear=" + String(linear, 3) +
             " angular=" + String(angular, 3))
                .c_str());

        return;
    }

    Logger::warn("BACKEND", "Unknown WS command (ignored)");
}

IPAddress BackendClient::computeBroadcastAddress()
{
    IPAddress ip = WiFi.localIP();
    IPAddress mask = WiFi.subnetMask();

    IPAddress broadcast;
    for (int i = 0; i < 4; i++)
        broadcast[i] = ip[i] | (~mask[i]);

    return broadcast;
}

void BackendClient::sendUdpAnnounce()
{
    IPAddress broadcast = computeBroadcastAddress();
    String payload = "{\"type\":\"announce\",\"port\":" + String(_robotHttpPort) + "}";

    _udp.beginPacket(broadcast, _udpPort);
    _udp.print(payload);
    _udp.endPacket();

    Logger::info("BACKEND", "UDP announce sent");
}
