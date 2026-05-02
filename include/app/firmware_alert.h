#pragma once

#include <Arduino.h>

namespace FirmwareAlert
{
    enum class Severity : uint8_t
    {
        Info,
        Warn,
        Error
    };

    bool send(Severity severity, const String &message);

    bool info(const String &message);
    bool warn(const String &message);
    bool error(const String &message);

    bool infof(const char *format, ...);
    bool warnf(const char *format, ...);
    bool errorf(const char *format, ...);
}
