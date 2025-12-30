#include "api_handler.h"
#include <ArduinoJson.h>

ApiHandler::ApiHandler(AsyncWebServer *server, SystemState *state)
    : _server(server), _state(state) {}

void ApiHandler::begin()
{
    setupRoutes();
}

void ApiHandler::setupRoutes()
{
    // GET /status - Telemetry endpoint
    _server->on("/status", HTTP_GET, [this](AsyncWebServerRequest *request)
                {
        JsonDocument doc;
        doc["systemHealth"] = _state->systemHealth;
        doc["batteryLevel"] = _state->batteryLevel;
        doc["driveMode"] = (_state->driveMode == MANUAL) ? "MANUAL" : 
                           (_state->driveMode == AUTO) ? "AUTO" : "IDLE";
        doc["position"] = _state->currentPosition;

        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response); });

    // POST /mode - Change robot mode
    _server->on("/mode", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
                {
            JsonDocument doc;
            deserializeJson(doc, data);
            
            if (!doc["mode"].isNull()) { 
                String mode = doc["mode"];
                if (mode == "MANUAL") _state->driveMode = MANUAL;
                else if (mode == "IDLE") _state->driveMode = IDLE;
                else if (mode == "AUTO") _state->driveMode = AUTO;
                
                request->send(200, "application/json", "{\"status\":\"success\"}");
            } else {
                request->send(400, "application/json", "{\"status\":\"error\"}");
            } });
}