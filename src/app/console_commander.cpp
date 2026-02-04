#include "app/console_commander.h"

#include "app/app_utils.h"

ConsoleCommander::ConsoleCommander(DriveController &driveRef, SensorSuite &sensorsRef, LedController &ledsRef, I2sAudio &audioRef)
    : drive(driveRef), sensors(sensorsRef), leds(ledsRef), audio(audioRef)
{
}

void ConsoleCommander::begin()
{
    console.begin();
}

void ConsoleCommander::printHelp()
{
    Serial.println("Commands:");
    Serial.println("  help                      - show commands");
    Serial.println("  l <v>                     - set left motor [-1.0..1.0] (direct, no smoothing)");
    Serial.println("  r <v>                     - set right motor [-1.0..1.0] (direct, no smoothing)");
    Serial.println("  tank <throttle> <steer>   - set targets [-1.0..1.0] (smoothed)");
    Serial.println("  stop                      - stop both motors");
    Serial.println("  drivedbg on|off            - drive debug prints");
    Serial.println("  ir                         - print IR sensor states once");
    Serial.println("  irwatch on|off              - print IR edge events");
    Serial.println("  irperiodic on|off           - periodic IR snapshot every 500ms");
    Serial.println("  lux                        - print light sensor lux once");
    Serial.println("  luxperiodic on|off          - periodic lux print every 500ms");
    Serial.println("  led on|off                 - manual LED on/off");
    Serial.println("  ledauto on|off             - enable/disable auto LED control");
    Serial.println("  ledcolor <r> <g> <b>       - set LED color (0..255)");
    Serial.println("  ledbri <x>                 - set LED brightness (0..255)");
    Serial.println("  vol <0..100>               - set audio volume percent");
    Serial.println("  beep                       - play short test beep");
    Serial.println("  beep <hz> <ms>             - play beep with frequency and duration");
}

void ConsoleCommander::handle()
{
    String line;
    if (!console.pollLine(line))
        return;

    String trimmed = line;
    trimmed.trim();

    if (trimmed.equalsIgnoreCase("help"))
    {
        printHelp();
        return;
    }

    float a = 0.0f;
    float b = 0.0f;
    int ia = 0, ib = 0, ic = 0;

    if (sscanf(trimmed.c_str(), "l %f", &a) == 1)
    {
        drive.setLeftDirect(a);
        return;
    }
    if (sscanf(trimmed.c_str(), "r %f", &a) == 1)
    {
        drive.setRightDirect(a);
        return;
    }

    if (sscanf(trimmed.c_str(), "tank %f %f", &a, &b) == 2)
    {
        drive.setTargets(a, b, false);
        return;
    }

    if (trimmed.equalsIgnoreCase("stop"))
    {
        drive.setTargets(0.0f, 0.0f, true);
        return;
    }

    if (trimmed.startsWith("drivedbg"))
    {
        int sp = trimmed.indexOf(' ');
        if (sp > 0)
        {
            bool on = false;
            if (parseOnOffOr01(trimmed.substring(sp + 1), on))
            {
                drive.setDebug(on);
                Serial.printf("[drive] debug %s\n", on ? "on" : "off");
                return;
            }
        }
        Serial.println("[drive] usage: drivedbg on|off");
        return;
    }

    if (trimmed.equalsIgnoreCase("ir"))
    {
        sensors.printIrOnce();
        return;
    }

    if (trimmed.startsWith("irwatch"))
    {
        int sp = trimmed.indexOf(' ');
        if (sp > 0)
        {
            bool on = false;
            if (parseOnOffOr01(trimmed.substring(sp + 1), on))
            {
                sensors.setIrWatch(on);
                Serial.printf("[ir] watch %s\n", on ? "on" : "off");
                return;
            }
        }
        Serial.println("[ir] usage: irwatch on|off");
        return;
    }

    if (trimmed.startsWith("irperiodic"))
    {
        int sp = trimmed.indexOf(' ');
        if (sp > 0)
        {
            bool on = false;
            if (parseOnOffOr01(trimmed.substring(sp + 1), on))
            {
                sensors.setIrPeriodic(on);
                Serial.printf("[ir] periodic %s\n", on ? "on" : "off");
                return;
            }
        }
        Serial.println("[ir] usage: irperiodic on|off");
        return;
    }

    if (trimmed.equalsIgnoreCase("lux"))
    {
        sensors.printLuxOnce();
        return;
    }

    if (trimmed.startsWith("luxperiodic"))
    {
        int sp = trimmed.indexOf(' ');
        if (sp > 0)
        {
            bool on = false;
            if (parseOnOffOr01(trimmed.substring(sp + 1), on))
            {
                sensors.setLuxPeriodic(on);
                Serial.printf("[lux] periodic %s\n", on ? "on" : "off");
                return;
            }
        }
        Serial.println("[lux] usage: luxperiodic on|off");
        return;
    }

    if (trimmed.startsWith("ledauto"))
    {
        int sp = trimmed.indexOf(' ');
        if (sp > 0)
        {
            bool on = false;
            if (parseOnOffOr01(trimmed.substring(sp + 1), on))
            {
                leds.setAutoEnabled(on);
                Serial.printf("[led] auto %s\n", on ? "on" : "off");
                return;
            }
        }
        Serial.println("[led] usage: ledauto on|off");
        return;
    }

    if (trimmed.startsWith("led "))
    {
        int sp = trimmed.indexOf(' ');
        bool on = false;
        if (sp > 0 && parseOnOffOr01(trimmed.substring(sp + 1), on))
        {
            leds.setAutoEnabled(false);
            leds.setEnabled(on);
            return;
        }
    }

    if (sscanf(trimmed.c_str(), "ledcolor %d %d %d", &ia, &ib, &ic) == 3)
    {
        leds.setAutoEnabled(false);
        leds.setColor(static_cast<uint8_t>(clampf(ia, 0, 255)),
                      static_cast<uint8_t>(clampf(ib, 0, 255)),
                      static_cast<uint8_t>(clampf(ic, 0, 255)));
        leds.setEnabled(true);
        leds.apply();
        return;
    }

    if (sscanf(trimmed.c_str(), "ledbri %d", &ia) == 1)
    {
        leds.setBrightness(static_cast<uint8_t>(clampf(ia, 0, 255)));
        leds.apply();
        return;
    }

    if (sscanf(trimmed.c_str(), "vol %d", &ia) == 1)
    {
        ia = static_cast<int>(clampf(ia, 0, 100));
        audio.setVolume(static_cast<float>(ia) / 100.0f);
        Serial.printf("[audio] volume=%d%%\n", ia);
        return;
    }

    if (trimmed.equalsIgnoreCase("beep"))
    {
        audio.playBeep(880, 150);
        return;
    }

    if (sscanf(trimmed.c_str(), "beep %d %d", &ia, &ib) == 2)
    {
        ia = static_cast<int>(clampf(ia, 20, 20000));
        ib = static_cast<int>(clampf(ib, 10, 5000));
        audio.playBeep(static_cast<uint16_t>(ia), static_cast<uint16_t>(ib));
        return;
    }

    Serial.printf("[console] unknown: %s\n", trimmed.c_str());
}
