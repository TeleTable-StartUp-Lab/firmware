#include "backend_client.h"
#include <WiFi.h>
#include "utils/logger.h"

// TEMP: backend config placeholder (no backend running yet -> reconnect logs are expected)
static const char *BACKEND_HOST = "192.168.0.100";
static const uint16_t BACKEND_WS_PORT = 3003;

BackendClient::BackendClient(uint16_t udpPort, uint16_t robotHttpPort)
    : _udpPort(udpPort), _robotHttpPort(robotHttpPort) {}

void BackendClient::begin()
{
    _udp.begin(0);

    setupWebSocket();

    Logger::info("BACKEND", "BackendClient initialized (UDP + WS)");
    _lastAnnounceMs = 0; // force announce soon
}

void BackendClient::loop()
{
    if (WiFi.status() != WL_CONNECTED)
        return;

    // UDP announce
    const unsigned long now = millis();
    if (now - _lastAnnounceMs >= ANNOUNCE_INTERVAL_MS)
    {
        sendUdpAnnounce();
        _lastAnnounceMs = now;
    }

    // WebSocket
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
    (void)payload;
    (void)length;

    switch (type)
    {
    case WStype_CONNECTED:
        Logger::info("BACKEND", "WebSocket connected");
        break;

    case WStype_DISCONNECTED:
        Logger::warn("BACKEND", "WebSocket disconnected (will retry)");
        break;

    case WStype_TEXT:
        Logger::info("BACKEND", "WebSocket text received (ignored for now)");
        break;

    default:
        break;
    }
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
