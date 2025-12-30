#ifndef API_HANDLER_H
#define API_HANDLER_H

#include <ESPAsyncWebServer.h>
#include "core/system_state.h"

class ApiHandler
{
public:
    ApiHandler(AsyncWebServer *server, SystemState *state);
    void begin();

private:
    AsyncWebServer *_server;
    SystemState *_state;

    void setupRoutes();
};

#endif