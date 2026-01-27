#include "app/serial_console.h"

void SerialConsole::begin()
{
    buffer_.reserve(96);
}

bool SerialConsole::pollLine(String &outLine)
{
    while (Serial.available() > 0)
    {
        const char c = static_cast<char>(Serial.read());

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            outLine = buffer_;
            buffer_.clear();
            outLine.trim();
            return outLine.length() > 0;
        }

        if (buffer_.length() < 160)
        {
            buffer_ += c;
        }
    }
    return false;
}
