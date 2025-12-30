#include "logger.h"

void Logger::log(LogLevel level, const char *module, const char *message)
{
    // Format: [TIMESTAMP] [LEVEL] [MODULE] MESSAGE
    Serial.print("[");
    Serial.print(millis());
    Serial.print("] [");
    Serial.print(levelToString(level));
    Serial.print("] [");
    Serial.print(module);
    Serial.print("] ");
    Serial.println(message);
}

void Logger::info(const char *module, const char *message) { log(LOG_INFO, module, message); }
void Logger::warn(const char *module, const char *message) { log(LOG_WARN, module, message); }
void Logger::error(const char *module, const char *message) { log(LOG_ERROR, module, message); }
void Logger::debug(const char *module, const char *message) { log(LOG_DEBUG, module, message); }

const char *Logger::levelToString(LogLevel level)
{
    switch (level)
    {
    case LOG_INFO:
        return "INFO";
    case LOG_WARN:
        return "WARN";
    case LOG_ERROR:
        return "ERROR";
    case LOG_DEBUG:
        return "DEBUG";
    default:
        return "UNKNOWN";
    }
}