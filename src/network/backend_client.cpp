#include "backend_client.h"
#include <WiFi.h>
#include "utils/logger.h"

BackendClient::BackendClient(uint16_t udpPort, uint16_t robotHttpPort)
    : _udpPort(udpPort), _robotHttpPort(robotHttpPort) {}

void BackendClient::begin()
{
    // Bind to an ephemeral local port for sending
    _udp.begin(0);

    String msg = "BackendClient UDP announce enabled. Target UDP port: " + String(_udpPort);
    Logger::info("BACKEND", msg.c_str());

    // Send once immediately on startup
    _lastAnnounceMs = 0;
}

void BackendClient::loop()
{
    if (WiFi.status() != WL_CONNECTED)
        return;

    const unsigned long now = millis();
    if (now - _lastAnnounceMs >= _announceIntervalMs)
    {
        sendUdpAnnounce();
        _lastAnnounceMs = now;
    }
}

IPAddress BackendClient::computeBroadcastAddress()
{
    IPAddress ip = WiFi.localIP();
    IPAddress mask = WiFi.subnetMask();

    // broadcast = ip | (~mask)
    IPAddress broadcast;
    for (int i = 0; i < 4; i++)
    {
        broadcast[i] = ip[i] | (~mask[i]);
    }
    return broadcast;
}

void BackendClient::sendUdpAnnounce()
{
    IPAddress broadcast = computeBroadcastAddress();

    String payload = "{\"type\":\"announce\",\"port\":" + String(_robotHttpPort) + "}";

    _udp.beginPacket(broadcast, _udpPort);
    _udp.print(payload);
    _udp.endPacket();

    String msg = "UDP announce sent to " + broadcast.toString() + ":" + String(_udpPort) + " payload=" + payload;
    Logger::info("BACKEND", msg.c_str());
}
