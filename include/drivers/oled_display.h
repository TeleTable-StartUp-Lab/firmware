#pragma once
#include <Arduino.h>
#include <Wire.h>

class OledDisplay
{
public:
    struct Config
    {
        TwoWire *wire;
        uint8_t address;
        uint8_t width;
        uint8_t height;
        int8_t reset_pin; // -1 if not used
    };

    explicit OledDisplay(const Config &cfg);

    bool begin();
    bool isOk() const;

    uint8_t maxLines() const;

    void clear();
    void setLine(uint8_t index, const String &text);
    void setLines(const String *lines, uint8_t count);
    void show();

private:
    Config cfg_;
    bool ok_ = false;

    String lines_[8];

    void trimToFit_(String &s) const;
};
