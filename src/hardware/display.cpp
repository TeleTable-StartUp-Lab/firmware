#include "display.h"
#include "pins.h"
#include "utils/logger.h"
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

DisplayController::DisplayController()
    : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}

bool DisplayController::begin()
{
    Wire.begin(I2C_SDA, I2C_SCL);

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Logger::error("DISPLAY", "SSD1306 allocation failed");
        return false;
    }

    Logger::info("DISPLAY", "OLED initialized");
    clear();
    return true;
}

void DisplayController::clear()
{
    display.clearDisplay();
    display.display();
}

void DisplayController::showText(const char *text, int x, int y, int size)
{
    display.setTextSize(size);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(x, y);
    display.print(text);
    display.display(); // Instantly updates the screen
}

void DisplayController::drawHeader(const char *title)
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(title);
    display.drawFastHLine(0, 10, 128, SSD1306_WHITE); // A dividing line
    display.display();
}