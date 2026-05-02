#include "app/firmware_alert.h"

#include "net/backend_client.h"

#include <cstdarg>
#include <cstdio>

namespace
{
    constexpr uint32_t DUPLICATE_SUPPRESS_MS = 5000;
    constexpr size_t ALERT_BUFFER_CAP = 160;

    FirmwareAlert::Severity g_lastSeverity = FirmwareAlert::Severity::Info;
    String g_lastMessage;
    uint32_t g_lastSentMs = 0;

    const char *severityToString(FirmwareAlert::Severity severity)
    {
        switch (severity)
        {
        case FirmwareAlert::Severity::Info:
            return "INFO";
        case FirmwareAlert::Severity::Warn:
            return "WARN";
        case FirmwareAlert::Severity::Error:
            return "ERROR";
        default:
            return "INFO";
        }
    }

    bool shouldSuppress(FirmwareAlert::Severity severity, const String &message, uint32_t nowMs)
    {
        if (message.length() == 0)
            return true;

        if (severity != g_lastSeverity || message != g_lastMessage)
            return false;

        return (nowMs - g_lastSentMs) < DUPLICATE_SUPPRESS_MS;
    }

    bool sendFormatted(FirmwareAlert::Severity severity, const char *format, va_list args)
    {
        char buffer[ALERT_BUFFER_CAP];
        vsnprintf(buffer, sizeof(buffer), format ? format : "", args);
        return FirmwareAlert::send(severity, String(buffer));
    }
}

namespace FirmwareAlert
{
    bool send(Severity severity, const String &message)
    {
        const uint32_t nowMs = millis();
        if (shouldSuppress(severity, message, nowMs))
            return false;

        Serial.printf("[alert] %s %s\n", severityToString(severity), message.c_str());

        const bool queued = BackendClient::queueEvent(String(severityToString(severity)), message);
        if (queued)
        {
            g_lastSeverity = severity;
            g_lastMessage = message;
            g_lastSentMs = nowMs;
        }

        return queued;
    }

    bool info(const String &message)
    {
        return send(Severity::Info, message);
    }

    bool warn(const String &message)
    {
        return send(Severity::Warn, message);
    }

    bool error(const String &message)
    {
        return send(Severity::Error, message);
    }

    bool infof(const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        const bool ok = sendFormatted(Severity::Info, format, args);
        va_end(args);
        return ok;
    }

    bool warnf(const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        const bool ok = sendFormatted(Severity::Warn, format, args);
        va_end(args);
        return ok;
    }

    bool errorf(const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        const bool ok = sendFormatted(Severity::Error, format, args);
        va_end(args);
        return ok;
    }
}
