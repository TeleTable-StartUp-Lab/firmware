#ifndef BACKEND_CLIENT_H
#define BACKEND_CLIENT_H

#include <Arduino.h>
#include <WiFiUdp.h>
#include <WebSocketsClient.h> 

class BackendClient
{
public:
    BackendClient(uint16_t udpPort, uint16_t robotHttpPort);

    void begin();
    void loop();

private:
    WiFiUDP _udp;
    uint16_t _udpPort;
    uint16_t _robotHttpPort;

    unsigned long _lastAnnounceMs = 0;
    static constexpr unsigned long ANNOUNCE_INTERVAL_MS = 10000;

    IPAddress computeBroadcastAddress();
    void sendUdpAnnounce();

    WebSocketsClient _ws;

    void setupWebSocket();
    void onWebSocketEvent(WStype_t type, uint8_t *payload, size_t length);
};

#endif 
