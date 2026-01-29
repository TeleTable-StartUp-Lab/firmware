#include "drivers/oled_display.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

namespace
{
    constexpr uint8_t CHAR_W = 6; // 5px font + 1px spacing
    constexpr uint8_t CHAR_H = 8; // text size 1
}

class OledDisplayImpl
{
public:
    OledDisplayImpl(uint8_t w, uint8_t h, TwoWire *wire, int8_t rst)
        : ssd(w, h, wire, rst) {}

    Adafruit_SSD1306 ssd;
};

OledDisplay::OledDisplay(const Config &cfg) : cfg_(cfg)
{
    for (auto &l : lines_)
        l = "";
}

static OledDisplayImpl *g_impl = nullptr;

bool OledDisplay::begin()
{
    if (!cfg_.wire)
        return false;

    if (!g_impl)
        g_impl = new OledDisplayImpl(cfg_.width, cfg_.height, cfg_.wire, cfg_.reset_pin);

    ok_ = g_impl->ssd.begin(SSD1306_SWITCHCAPVCC, cfg_.address);
    if (!ok_)
        return false;

    g_impl->ssd.clearDisplay();
    g_impl->ssd.setTextSize(1);
    g_impl->ssd.setTextColor(SSD1306_WHITE);
    g_impl->ssd.setCursor(0, 0);
    g_impl->ssd.print("OLED ok");
    g_impl->ssd.display();
    return true;
}

bool OledDisplay::isOk() const { return ok_; }

uint8_t OledDisplay::maxLines() const
{
    const uint8_t lines = cfg_.height / CHAR_H;
    return (lines > 8) ? 8 : lines;
}

void OledDisplay::clear()
{
    for (auto &l : lines_)
        l = "";
    show();
}

void OledDisplay::setLine(uint8_t index, const String &text)
{
    if (index >= maxLines())
        return;

    lines_[index] = text;
    trimToFit_(lines_[index]);
}

void OledDisplay::setLines(const String *lines, uint8_t count)
{
    const uint8_t n = (count > maxLines()) ? maxLines() : count;

    for (uint8_t i = 0; i < n; ++i)
    {
        lines_[i] = lines[i];
        trimToFit_(lines_[i]);
    }
    for (uint8_t i = n; i < maxLines(); ++i)
        lines_[i] = "";
}

void OledDisplay::show()
{
    if (!ok_ || !g_impl)
        return;

    g_impl->ssd.clearDisplay();
    g_impl->ssd.setCursor(0, 0);

    const uint8_t n = maxLines();
    for (uint8_t i = 0; i < n; ++i)
    {
        g_impl->ssd.println(lines_[i]);
    }

    g_impl->ssd.display();
}

void OledDisplay::trimToFit_(String &s) const
{
    const uint8_t maxChars = cfg_.width / CHAR_W;
    if (s.length() > maxChars)
        s = s.substring(0, maxChars);
}
