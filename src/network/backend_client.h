#ifndef BACKEND_CLIENT_H
#define BACKEND_CLIENT_H

#include <Arduino.h>
#include <WiFiUdp.h>

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
    const unsigned long _announceIntervalMs = 10000;

    IPAddress computeBroadcastAddress();
    void sendUdpAnnounce();
};

#endif
