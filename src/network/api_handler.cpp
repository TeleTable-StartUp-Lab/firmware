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
    // GET /health - Healthcheck endpoint
    _server->on("/health", HTTP_GET, [](AsyncWebServerRequest *request)
                { request->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Robot is online\"}"); });

    // GET /nodes - Available navigation nodes
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

    // GET /status - Telemetry endpoint
    _server->on("/status", HTTP_GET, [this](AsyncWebServerRequest *request)
                {
                    Logger::debug("API", "Status requested via GET");
                    JsonDocument doc;

                    // System information
                    doc["systemHealth"] = _state->systemHealth;
                    doc["batteryLevel"] = _state->batteryLevel;
                    doc["driveMode"] = _state->driveModeToCString();
                    doc["position"] = _state->currentPosition;

                    // Sensor data
                    doc["lux"] = _state->lux;
                    doc["obstacleLeft"] = _state->obstacleLeft;
                    doc["obstacleRight"] = _state->obstacleRight;

                    // Manual control (debug)
                    doc["linearVelocity"] = _state->linearVelocity;
                    doc["angularVelocity"] = _state->angularVelocity;

                    String response;
                    serializeJson(doc, response);
                    request->send(200, "application/json", response); });

    // Shared POST handler for /mode and /control/mode
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

        if (_state->setDriveModeFromString(mode))
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

    // POST /mode - Change robot mode
    _server->on("/mode", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, modeHandler);

    // POST /control/mode - Alias for backend timeout handling
    _server->on("/control/mode", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, modeHandler);
}
