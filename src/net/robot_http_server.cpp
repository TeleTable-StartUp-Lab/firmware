#include "net/robot_http_server.h"

#include <WebServer.h>
#include <ArduinoJson.h>

namespace RobotHttpServer
{
    static WebServer *g_server = nullptr;

    static StatusProvider g_statusProvider;
    static ModeSetter g_modeSetter;
    static RouteSetter g_routeSetter;

    static const char *modeToStr(DriveMode m)
    {
        switch (m)
        {
        case DriveMode::IDLE:
            return "IDLE";
        case DriveMode::MANUAL:
            return "MANUAL";
        case DriveMode::AUTO:
            return "AUTO";
        default:
            return "IDLE";
        }
    }

    static bool strToMode(const String &s, DriveMode &out)
    {
        if (s == "IDLE")
        {
            out = DriveMode::IDLE;
            return true;
        }
        if (s == "MANUAL")
        {
            out = DriveMode::MANUAL;
            return true;
        }
        if (s == "AUTO")
        {
            out = DriveMode::AUTO;
            return true;
        }
        return false;
    }

    static void sendJson(int code, JsonDocument &doc)
    {
        if (!g_server)
            return;

        String body;
        serializeJson(doc, body);
        g_server->send(code, "application/json", body);
    }

    static void handleStatus()
    {
        if (!g_server)
            return;

        if (!g_statusProvider)
        {
            JsonDocument doc;
            doc["ok"] = false;
            doc["error"] = "status provider not set";
            sendJson(500, doc);
            return;
        }

        const StatusSnapshot s = g_statusProvider();

        JsonDocument doc;

        doc["systemHealth"] = s.systemHealth ? s.systemHealth : "UNKNOWN";
        doc["batteryLevel"] = s.batteryLevel;
        doc["driveMode"] = modeToStr(s.driveMode);
        doc["cargoStatus"] = s.cargoStatus ? s.cargoStatus : "UNKNOWN";

        JsonObject lastRoute = doc["lastRoute"].to<JsonObject>();
        lastRoute["startNode"] = s.lastRouteStart ? s.lastRouteStart : "";
        lastRoute["endNode"] = s.lastRouteEnd ? s.lastRouteEnd : "";

        doc["position"] = s.position ? s.position : "";

        JsonObject sensors = doc["sensors"].to<JsonObject>();

        JsonObject ir = sensors["ir"].to<JsonObject>();
        ir["left"] = s.irLeft;
        ir["middle"] = s.irMid;
        ir["right"] = s.irRight;

        JsonObject light = sensors["light"].to<JsonObject>();
        light["luxValid"] = s.luxValid;
        light["lux"] = s.luxValid ? s.lux : 0.0f;

        JsonObject led = doc["led"].to<JsonObject>();
        led["enabled"] = s.ledEnabled;
        led["autoEnabled"] = s.ledAutoEnabled;

        JsonObject audio = doc["audio"].to<JsonObject>();
        audio["volume"] = s.audioVolume;

        sendJson(200, doc);
    }

    static void handleNodes()
    {
        if (!g_server)
            return;

        JsonDocument doc;
        JsonArray nodes = doc["nodes"].to<JsonArray>();
        nodes.add("Mensa");
        nodes.add("Raum #1");
        nodes.add("Raum #3");

        sendJson(200, doc);
    }

    static void handleMode()
    {
        if (!g_server)
            return;

        if (!g_server->hasArg("plain"))
        {
            JsonDocument doc;
            doc["ok"] = false;
            doc["error"] = "missing body";
            sendJson(400, doc);
            return;
        }

        JsonDocument in;
        const DeserializationError err = deserializeJson(in, g_server->arg("plain"));
        if (err)
        {
            JsonDocument doc;
            doc["ok"] = false;
            doc["error"] = "invalid json";
            sendJson(400, doc);
            return;
        }

        const String modeStr = in["mode"] | "";
        DriveMode mode;
        if (!strToMode(modeStr, mode))
        {
            JsonDocument doc;
            doc["ok"] = false;
            doc["error"] = "invalid mode";
            sendJson(400, doc);
            return;
        }

        if (g_modeSetter)
            g_modeSetter(mode);

        JsonDocument doc;
        doc["ok"] = true;
        doc["mode"] = modeStr;
        sendJson(200, doc);
    }

    static void handleSelect()
    {
        if (!g_server)
            return;

        if (!g_server->hasArg("plain"))
        {
            JsonDocument doc;
            doc["ok"] = false;
            doc["error"] = "missing body";
            sendJson(400, doc);
            return;
        }

        JsonDocument in;
        const DeserializationError err = deserializeJson(in, g_server->arg("plain"));
        if (err)
        {
            JsonDocument doc;
            doc["ok"] = false;
            doc["error"] = "invalid json";
            sendJson(400, doc);
            return;
        }

        const String startNode = in["startNode"] | "";
        const String endNode = in["endNode"] | "";

        if (startNode.length() == 0 || endNode.length() == 0)
        {
            JsonDocument doc;
            doc["ok"] = false;
            doc["error"] = "startNode/endNode required";
            sendJson(400, doc);
            return;
        }

        if (g_routeSetter)
            g_routeSetter(startNode, endNode);

        JsonDocument doc;
        doc["ok"] = true;
        doc["startNode"] = startNode;
        doc["endNode"] = endNode;
        sendJson(200, doc);
    }

    void begin(uint16_t port, StatusProvider statusProvider, ModeSetter modeSetter, RouteSetter routeSetter)
    {
        g_statusProvider = statusProvider;
        g_modeSetter = modeSetter;
        g_routeSetter = routeSetter;

        if (g_server)
        {
            Serial.println("[robot-http] already started");
            return;
        }

        g_server = new WebServer(port);

        g_server->on("/status", HTTP_GET, handleStatus);
        g_server->on("/nodes", HTTP_GET, handleNodes);
        g_server->on("/mode", HTTP_POST, handleMode);
        g_server->on("/select", HTTP_POST, handleSelect);

        g_server->onNotFound([]()
                             {
                                 JsonDocument doc;
                                 doc["ok"] = false;
                                 doc["error"] = "not found";
                                 doc["path"] = g_server ? g_server->uri() : "";
                                 sendJson(404, doc); });

        g_server->begin();
        Serial.printf("[robot-http] listening on port %u\n", static_cast<unsigned>(port));
    }

    void handle()
    {
        if (!g_server)
            return;
        g_server->handleClient();
    }

}
