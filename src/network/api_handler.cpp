#include "api_handler.h"
#include <ArduinoJson.h>
#include "utils/logger.h"

ApiHandler::ApiHandler(AsyncWebServer *server, SystemState *state)
    : _server(server), _state(state) {}

void ApiHandler::begin()
{
    setupRoutes();
    Logger::info("API", "REST API routes initialized");
}

void ApiHandler::setupRoutes()
{
    _server->on("/health", HTTP_GET, [](AsyncWebServerRequest *request)
                { request->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Robot is online\"}"); });

    _server->on("/nodes", HTTP_GET, [](AsyncWebServerRequest *request)
                {
                    JsonDocument doc;
                    JsonArray nodes = doc["nodes"].to<JsonArray>();

                    nodes.add("Home");
                    nodes.add("Kitchen");
                    nodes.add("Living Room");
                    nodes.add("Office");
                    nodes.add("Bedroom");
                    nodes.add("Charging Station");

                    String response;
                    serializeJson(doc, response);
                    request->send(200, "application/json", response); });

    _server->on("/status", HTTP_GET, [this](AsyncWebServerRequest *request)
                {
                    Logger::debug("API", "Status requested via GET");
                    JsonDocument doc;

                    String systemHealth, position;
                    int batteryLevel;
                    float lux, linear, angular;
                    bool obsL, obsR;
                    const char *modeStr;

                    _state->lock();
                    systemHealth = _state->systemHealth;
                    batteryLevel = _state->batteryLevel;
                    modeStr = _state->driveModeToCString();
                    position = _state->currentPosition;

                    lux = _state->lux;
                    obsL = _state->obstacleLeft;
                    obsR = _state->obstacleRight;

                    linear = _state->linearVelocity;
                    angular = _state->angularVelocity;
                    _state->unlock();

                    doc["systemHealth"] = systemHealth;
                    doc["batteryLevel"] = batteryLevel;
                    doc["driveMode"] = modeStr;
                    doc["position"] = position;
                    doc["lux"] = lux;
                    doc["obstacleLeft"] = obsL;
                    doc["obstacleRight"] = obsR;
                    doc["linearVelocity"] = linear;
                    doc["angularVelocity"] = angular;

                    String response;
                    serializeJson(doc, response);
                    request->send(200, "application/json", response); });

    auto modeHandler = [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
    {
        (void)index;
        (void)total;

        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, data, len);

        if (err)
        {
            Logger::warn("API", "Failed to parse JSON in mode endpoint");
            request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"invalid_json\"}");
            return;
        }

        String mode = doc["mode"] | "";
        if (mode.length() == 0)
        {
            Logger::warn("API", "Mode missing in request body");
            request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"missing_mode\"}");
            return;
        }

        bool ok = false;
        _state->lock();
        ok = _state->setDriveModeFromString(mode);
        _state->unlock();

        if (ok)
        {
            Logger::info("API", ("Drive mode changed via POST to " + mode).c_str());
            request->send(200, "application/json", "{\"status\":\"success\"}");
        }
        else
        {
            Logger::warn("API", ("Invalid drive mode received: " + mode).c_str());
            request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"invalid_mode\"}");
        }
    };

    _server->on("/mode", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, modeHandler);
    _server->on("/control/mode", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, modeHandler);
}
