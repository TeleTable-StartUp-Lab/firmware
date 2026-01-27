#pragma once
#include <Arduino.h>

class SerialConsole
{
public:
    void begin();
    bool pollLine(String &outLine);

private:
    String buffer_;
};
