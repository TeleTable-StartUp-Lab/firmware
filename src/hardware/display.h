#ifndef DISPLAY_HANDLER_H
#define DISPLAY_HANDLER_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class DisplayController
{
public:
    DisplayController();
    bool begin();

    void clear();
    void showText(const char *text, int x = 0, int y = 0, int size = 1);
    void drawHeader(const char *title);

    Adafruit_SSD1306 &getRawDisplay() { return display; }

private:
    Adafruit_SSD1306 display;
};

#endif