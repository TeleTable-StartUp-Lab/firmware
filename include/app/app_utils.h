#pragma once

#include <Arduino.h>
#include <cmath>

inline float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

inline bool approxEqual(float a, float b, float eps)
{
    return std::fabs(a - b) <= eps;
}

inline float slewTowards(float current, float target, float maxDelta)
{
    const float delta = target - current;
    if (delta > maxDelta)
        return current + maxDelta;
    if (delta < -maxDelta)
        return current - maxDelta;
    return target;
}

inline bool parseOnOffOr01(const String &s, bool &out)
{
    String t = s;
    t.trim();
    t.toLowerCase();
    if (t == "on" || t == "true" || t == "1")
    {
        out = true;
        return true;
    }
    if (t == "off" || t == "false" || t == "0")
    {
        out = false;
        return true;
    }
    return false;
}
