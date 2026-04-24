#include "net/backend_client.h"
#include "net/backend_config.h"
#include "secrets.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <ping/ping_sock.h>

namespace
{
    constexpr uint8_t REQUEST_QUEUE_LENGTH = 8;
    constexpr uint32_t WORKER_IDLE_MS = 25;
    constexpr size_t EVENT_NAME_CAP = 64;
    constexpr size_t TEXT_FIELD_CAP = 64;
    constexpr size_t SHORT_TEXT_FIELD_CAP = 24;
    constexpr size_t RFID_UID_CAP = 48;

    struct PingContext
    {
        volatile bool finished;
        volatile uint32_t minTimeMs;
        volatile uint32_t maxTimeMs;
        volatile uint32_t sumTimeMs;
    };

    enum class RequestType : uint8_t
    {
        Register,
        Event,
    };

    struct BackendRequest
    {
        RequestType type;
        uint16_t robotPort;
        char eventName[EVENT_NAME_CAP];
    };

    struct StatePayload
    {
        int batteryLevel;
        char systemHealth[SHORT_TEXT_FIELD_CAP];
        char driveMode[SHORT_TEXT_FIELD_CAP];
        char cargoStatus[SHORT_TEXT_FIELD_CAP];
        char currentPosition[TEXT_FIELD_CAP];
        char lastNode[TEXT_FIELD_CAP];
        char targetNode[TEXT_FIELD_CAP];
        bool hasGyroscope;
        float gyroXDps;
        float gyroYDps;
        float gyroZDps;
        bool hasRfid;
        char lastReadUuid[RFID_UID_CAP];
        bool hasLux;
        float lux;
        bool hasInfrared;
        bool infraredFront;
        bool infraredLeft;
        bool infraredRight;
        bool hasPower;
        float voltageV;
        float currentA;
        float powerW;
    };

    QueueHandle_t g_requestQueue = nullptr;
    TaskHandle_t g_workerTask = nullptr;
    portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;
    StatePayload g_statePayload{};
    uint32_t g_stateSequence = 0;
    bool g_statePending = false;

    void copyStringField(char *dest, size_t destSize, const String &value)
    {
        if (!dest || destSize == 0)
            return;

        const size_t n = value.length();
        const size_t copyLen = (n < (destSize - 1)) ? n : (destSize - 1);
        memcpy(dest, value.c_str(), copyLen);
        dest[copyLen] = '\0';
    }

    void onPingSuccess(esp_ping_handle_t hdl, void *args)
    {
        auto *ctx = static_cast<PingContext *>(args);

        uint16_t seqno = 0;
        uint8_t ttl = 0;
        uint32_t elapsedMs = 0;
        uint32_t packetSize = 0;
        ip_addr_t replyAddr;
        char addrBuf[IPADDR_STRLEN_MAX] = {0};

        esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
        esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
        esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsedMs, sizeof(elapsedMs));
        esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &packetSize, sizeof(packetSize));
        esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &replyAddr, sizeof(replyAddr));

        ipaddr_ntoa_r(&replyAddr, addrBuf, sizeof(addrBuf));

        if (ctx->minTimeMs == 0 || elapsedMs < ctx->minTimeMs)
            ctx->minTimeMs = elapsedMs;
        if (elapsedMs > ctx->maxTimeMs)
            ctx->maxTimeMs = elapsedMs;
        ctx->sumTimeMs += elapsedMs;

        Serial.printf("%lu bytes from %s: icmp_seq=%u ttl=%u time=%lums\n",
                      static_cast<unsigned long>(packetSize),
                      addrBuf,
                      static_cast<unsigned>(seqno),
                      static_cast<unsigned>(ttl),
                      static_cast<unsigned long>(elapsedMs));
    }

    void onPingTimeout(esp_ping_handle_t hdl, void *)
    {
        uint16_t seqno = 0;
        esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
        Serial.printf("Request timeout for icmp_seq %u\n", static_cast<unsigned>(seqno));
    }

    void onPingEnd(esp_ping_handle_t, void *args)
    {
        auto *ctx = static_cast<PingContext *>(args);
        ctx->finished = true;
    }

    String backendUrl(const char *path)
    {
        return String(BackendConfig::USE_TLS ? "https://" : "http://") +
               BackendConfig::HOST + ":" + String(BackendConfig::PORT) + String(path);
    }

    bool postJsonBlocking(const char *path, const String &jsonBody)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("[backend] request failed: wifi not connected");
            return false;
        }

        HTTPClient http;
        const String url = backendUrl(path);

        if (BackendConfig::USE_TLS)
        {
            WiFiClientSecure client;
            if (BackendConfig::TLS_INSECURE)
                client.setInsecure();
            else if (BackendConfig::TLS_CA_CERT)
                client.setCACert(BackendConfig::TLS_CA_CERT);

            http.begin(client, url);
            http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
            http.setRedirectLimit(3);
            http.setTimeout(3000);
            http.addHeader("Content-Type", "application/json");
            http.addHeader("X-Api-Key", Secrets::ROBOT_API_KEY);

            const int code = http.POST(jsonBody);
            String resp = http.getString();
            http.end();

            Serial.printf("[backend] POST %s code=%d resp=%s\n", path, code, resp.c_str());
            return code >= 200 && code < 300;
        }

        WiFiClient client;
        http.begin(client, url);
        http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
        http.setRedirectLimit(3);
        http.setTimeout(3000);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("X-Api-Key", Secrets::ROBOT_API_KEY);

        const int code = http.POST(jsonBody);
        String resp = http.getString();
        http.end();

        Serial.printf("[backend] POST %s code=%d resp=%s\n", path, code, resp.c_str());
        return code >= 200 && code < 300;
    }

    bool registerRobotBlocking(uint16_t robotPort)
    {
        JsonDocument doc;
        doc["port"] = robotPort;
        String payload;
        serializeJson(doc, payload);
        return postJsonBlocking("/table/register", payload);
    }

    bool postStateBlocking(const String &systemHealth,
                           int batteryLevel,
                           const String &driveMode,
                           const String &cargoStatus,
                           const String &currentPosition,
                           const String &lastNode,
                           const String &targetNode,
                           bool hasGyroscope,
                           float gyroXDps,
                           float gyroYDps,
                           float gyroZDps,
                           bool hasRfid,
                           const String &lastReadUuid,
                           bool hasLux,
                           float lux,
                           bool hasInfrared,
                           bool infraredFront,
                           bool infraredLeft,
                           bool infraredRight,
                           bool hasPower,
                           float voltageV,
                           float currentA,
                           float powerW)
    {
        JsonDocument doc;
        doc["systemHealth"] = systemHealth;
        doc["batteryLevel"] = batteryLevel;
        doc["driveMode"] = driveMode;
        doc["cargoStatus"] = cargoStatus;
        doc["currentPosition"] = currentPosition;
        doc["lastNode"] = lastNode;
        doc["targetNode"] = targetNode;

        if (hasGyroscope)
        {
            JsonObject gyroscope = doc["gyroscope"].to<JsonObject>();
            gyroscope["xDps"] = gyroXDps;
            gyroscope["yDps"] = gyroYDps;
            gyroscope["zDps"] = gyroZDps;
        }

        if (hasRfid)
            doc["lastReadUuid"] = lastReadUuid;

        if (hasLux)
            doc["lux"] = lux;

        if (hasInfrared)
        {
            JsonObject infrared = doc["infrared"].to<JsonObject>();
            infrared["front"] = infraredFront;
            infrared["left"] = infraredLeft;
            infrared["right"] = infraredRight;
        }

        if (hasPower)
        {
            doc["voltageV"] = voltageV;
            doc["currentA"] = currentA;
            doc["powerW"] = powerW;
        }

        String payload;
        serializeJson(doc, payload);
        return postJsonBlocking("/table/state", payload);
    }

    bool postEventBlocking(const String &eventName)
    {
        JsonDocument doc;
        doc["event"] = eventName;
        doc["timestamp"] = (uint32_t)millis();

        String payload;
        serializeJson(doc, payload);
        return postJsonBlocking("/table/event", payload);
    }

    bool tryTakePendingState(StatePayload &out, uint32_t &sequence)
    {
        bool hasState = false;

        portENTER_CRITICAL(&g_stateMux);
        if (g_statePending)
        {
            out = g_statePayload;
            sequence = g_stateSequence;
            hasState = true;
        }
        portEXIT_CRITICAL(&g_stateMux);

        return hasState;
    }

    void finishPendingState(uint32_t sequence, bool success)
    {
        portENTER_CRITICAL(&g_stateMux);
        if (success && g_statePending && g_stateSequence == sequence)
            g_statePending = false;
        portEXIT_CRITICAL(&g_stateMux);
    }

    void backendWorkerTask(void *)
    {
        for (;;)
        {
            BackendRequest request{};
            if (g_requestQueue && xQueueReceive(g_requestQueue, &request, pdMS_TO_TICKS(WORKER_IDLE_MS)) == pdTRUE)
            {
                switch (request.type)
                {
                case RequestType::Register:
                    registerRobotBlocking(request.robotPort);
                    break;

                case RequestType::Event:
                    postEventBlocking(String(request.eventName));
                    break;
                }
                continue;
            }

            StatePayload state{};
            uint32_t sequence = 0;
            if (tryTakePendingState(state, sequence))
            {
                const bool ok = postStateBlocking(String(state.systemHealth),
                                                  state.batteryLevel,
                                                  String(state.driveMode),
                                                  String(state.cargoStatus),
                                                  String(state.currentPosition),
                                                  String(state.lastNode),
                                                  String(state.targetNode),
                                                  state.hasGyroscope,
                                                  state.gyroXDps,
                                                  state.gyroYDps,
                                                  state.gyroZDps,
                                                  state.hasRfid,
                                                  String(state.lastReadUuid),
                                                  state.hasLux,
                                                  state.lux,
                                                  state.hasInfrared,
                                                  state.infraredFront,
                                                  state.infraredLeft,
                                                  state.infraredRight,
                                                  state.hasPower,
                                                  state.voltageV,
                                                  state.currentA,
                                                  state.powerW);
                finishPendingState(sequence, ok);
                continue;
            }

            vTaskDelay(pdMS_TO_TICKS(WORKER_IDLE_MS));
        }
    }
}

namespace BackendClient
{
    void begin()
    {
        if (!g_requestQueue)
            g_requestQueue = xQueueCreate(REQUEST_QUEUE_LENGTH, sizeof(BackendRequest));

        if (!g_workerTask && g_requestQueue)
            xTaskCreatePinnedToCore(backendWorkerTask, "backend-http", 6144, nullptr, 1, &g_workerTask, tskNO_AFFINITY);
    }

    PingResult ping()
    {
        PingResult result;
        result.wifiConnected = (WiFi.status() == WL_CONNECTED);
        result.targetValid = false;
        result.sessionStarted = false;
        result.sessionFinished = false;
        result.transmitted = 0;
        result.received = 0;
        result.durationMs = 0;
        result.minTimeMs = 0;
        result.maxTimeMs = 0;
        result.avgTimeMs = 0;

        if (!result.wifiConnected)
            return result;

        esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();
        config.count = 4;
        config.interval_ms = 1000;
        config.timeout_ms = 1000;

        if (!ipaddr_aton(BackendConfig::HOST, &config.target_addr))
            return result;

        result.targetValid = true;

        PingContext ctx{};
        esp_ping_handle_t pingHandle = nullptr;
        const esp_ping_callbacks_t callbacks{
            .cb_args = &ctx,
            .on_ping_success = onPingSuccess,
            .on_ping_timeout = onPingTimeout,
            .on_ping_end = onPingEnd};

        if (esp_ping_new_session(&config, &callbacks, &pingHandle) != ESP_OK)
            return result;

        result.sessionStarted = true;

        if (esp_ping_start(pingHandle) != ESP_OK)
        {
            esp_ping_delete_session(pingHandle);
            return result;
        }

        const uint32_t waitStartMs = millis();
        const uint32_t waitTimeoutMs = (config.count * config.interval_ms) + config.timeout_ms + 500;
        while (!ctx.finished && (millis() - waitStartMs) < waitTimeoutMs)
            delay(10);

        if (!ctx.finished)
        {
            esp_ping_stop(pingHandle);
        }

        uint32_t requestCount = 0;
        uint32_t replyCount = 0;
        uint32_t durationMs = 0;
        esp_ping_get_profile(pingHandle, ESP_PING_PROF_REQUEST, &requestCount, sizeof(requestCount));
        esp_ping_get_profile(pingHandle, ESP_PING_PROF_REPLY, &replyCount, sizeof(replyCount));
        esp_ping_get_profile(pingHandle, ESP_PING_PROF_DURATION, &durationMs, sizeof(durationMs));

        result.sessionFinished = ctx.finished;
        result.transmitted = requestCount;
        result.received = replyCount;
        result.durationMs = durationMs;
        result.minTimeMs = ctx.minTimeMs;
        result.maxTimeMs = ctx.maxTimeMs;
        result.avgTimeMs = (replyCount > 0) ? (ctx.sumTimeMs / replyCount) : 0;

        esp_ping_delete_session(pingHandle);

        return result;
    }

    bool registerRobot(uint16_t robotPort)
    {
        return registerRobotBlocking(robotPort);
    }

    bool queueRegisterRobot(uint16_t robotPort)
    {
        begin();
        if (!g_requestQueue)
            return false;

        BackendRequest request{};
        request.type = RequestType::Register;
        request.robotPort = robotPort;
        return xQueueSendToBack(g_requestQueue, &request, 0) == pdTRUE;
    }

    bool postState(const String &systemHealth,
                   int batteryLevel,
                   const String &driveMode,
                   const String &cargoStatus,
                   const String &currentPosition,
                   const String &lastNode,
                   const String &targetNode,
                   bool hasGyroscope,
                   float gyroXDps,
                   float gyroYDps,
                   float gyroZDps,
                   bool hasRfid,
                   const String &lastReadUuid,
                   bool hasLux,
                   float lux,
                   bool hasInfrared,
                   bool infraredFront,
                   bool infraredLeft,
                   bool infraredRight,
                   bool hasPower,
                   float voltageV,
                   float currentA,
                   float powerW)
    {
        return postStateBlocking(systemHealth,
                                 batteryLevel,
                                 driveMode,
                                 cargoStatus,
                                 currentPosition,
                                 lastNode,
                                 targetNode,
                                 hasGyroscope,
                                 gyroXDps,
                                 gyroYDps,
                                 gyroZDps,
                                 hasRfid,
                                 lastReadUuid,
                                 hasLux,
                                 lux,
                                 hasInfrared,
                                 infraredFront,
                                 infraredLeft,
                                 infraredRight,
                                 hasPower,
                                 voltageV,
                                 currentA,
                                 powerW);
    }

    bool queueState(const String &systemHealth,
                    int batteryLevel,
                    const String &driveMode,
                    const String &cargoStatus,
                    const String &currentPosition,
                    const String &lastNode,
                    const String &targetNode,
                    bool hasGyroscope,
                    float gyroXDps,
                    float gyroYDps,
                    float gyroZDps,
                    bool hasRfid,
                    const String &lastReadUuid,
                    bool hasLux,
                    float lux,
                    bool hasInfrared,
                    bool infraredFront,
                    bool infraredLeft,
                    bool infraredRight,
                    bool hasPower,
                    float voltageV,
                    float currentA,
                    float powerW)
    {
        begin();

        StatePayload nextState{};
        nextState.batteryLevel = batteryLevel;
        copyStringField(nextState.systemHealth, sizeof(nextState.systemHealth), systemHealth);
        copyStringField(nextState.driveMode, sizeof(nextState.driveMode), driveMode);
        copyStringField(nextState.cargoStatus, sizeof(nextState.cargoStatus), cargoStatus);
        copyStringField(nextState.currentPosition, sizeof(nextState.currentPosition), currentPosition);
        copyStringField(nextState.lastNode, sizeof(nextState.lastNode), lastNode);
        copyStringField(nextState.targetNode, sizeof(nextState.targetNode), targetNode);
        nextState.hasGyroscope = hasGyroscope;
        nextState.gyroXDps = gyroXDps;
        nextState.gyroYDps = gyroYDps;
        nextState.gyroZDps = gyroZDps;
        nextState.hasRfid = hasRfid;
        copyStringField(nextState.lastReadUuid, sizeof(nextState.lastReadUuid), lastReadUuid);
        nextState.hasLux = hasLux;
        nextState.lux = lux;
        nextState.hasInfrared = hasInfrared;
        nextState.infraredFront = infraredFront;
        nextState.infraredLeft = infraredLeft;
        nextState.infraredRight = infraredRight;
        nextState.hasPower = hasPower;
        nextState.voltageV = voltageV;
        nextState.currentA = currentA;
        nextState.powerW = powerW;

        portENTER_CRITICAL(&g_stateMux);
        g_statePayload = nextState;
        ++g_stateSequence;
        g_statePending = true;
        portEXIT_CRITICAL(&g_stateMux);

        return true;
    }

    bool postEvent(const String &eventName)
    {
        return postEventBlocking(eventName);
    }

    bool queueEvent(const String &eventName)
    {
        begin();
        if (!g_requestQueue)
            return false;

        BackendRequest request{};
        request.type = RequestType::Event;
        copyStringField(request.eventName, sizeof(request.eventName), eventName);
        return xQueueSendToBack(g_requestQueue, &request, 0) == pdTRUE;
    }
}
