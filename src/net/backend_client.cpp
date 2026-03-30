#include "net/backend_client.h"
#include "net/backend_config.h"
#include "secrets.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ping/ping_sock.h>

namespace
{
    struct PingContext
    {
        volatile bool finished;
        volatile uint32_t minTimeMs;
        volatile uint32_t maxTimeMs;
        volatile uint32_t sumTimeMs;
    };

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

    bool postJson(const char *path, const String &jsonBody)
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
}

namespace BackendClient
{
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
        JsonDocument doc;
        doc["port"] = robotPort;
        String payload;
        serializeJson(doc, payload);
        return postJson("/table/register", payload);
    }

    bool postState(const String &systemHealth,
                   int batteryLevel,
                   const String &driveMode,
                   const String &cargoStatus,
                   const String &currentPosition,
                   const String &lastNode,
                   const String &targetNode)
    {
        JsonDocument doc;
        doc["systemHealth"] = systemHealth;
        doc["batteryLevel"] = batteryLevel;
        doc["driveMode"] = driveMode;
        doc["cargoStatus"] = cargoStatus;
        doc["currentPosition"] = currentPosition;
        doc["lastNode"] = lastNode;
        doc["targetNode"] = targetNode;

        String payload;
        serializeJson(doc, payload);
        return postJson("/table/state", payload);
    }

    bool postEvent(const String &eventName)
    {
        JsonDocument doc;
        doc["event"] = eventName;
        doc["timestamp"] = (uint32_t)millis();

        String payload;
        serializeJson(doc, payload);
        return postJson("/table/event", payload);
    }
}
