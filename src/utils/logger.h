#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

// Log levels to categorize messages
enum LogLevel
{
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    LOG_DEBUG
};

class Logger
{
public:
    // Basic log function with level and module name
    static void log(LogLevel level, const char *module, const char *message);

    // Helper functions for easier usage
    static void info(const char *module, const char *message);
    static void warn(const char *module, const char *message);
    static void error(const char *module, const char *message);
    static void debug(const char *module, const char *message);

private:
    static const char *levelToString(LogLevel level);
};

#endif