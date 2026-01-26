#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>

#include "pins.h"
#include "utils/logger.h"

#include "core/system_state.h"
#include "core/runtime_tasks.h"

#include "network/wifi_manager.h"
#include "network/api_handler.h"
#include "network/backend_client.h"

#include "hardware/display.h"
#include "hardware/leds.h"
#include "hardware/bh1750.h"
#include "hardware/ir_sensors.h"
#include "hardware/motors.h"
#include "hardware/audio.h"

AsyncWebServer server(80);

SystemState state;
WifiManager wifi;
ApiHandler api(&server, &state);
BackendClient backend(3001, 80, &state);

DisplayController display;
LEDController leds;
BH1750Sensor lightSensor;

MotorController motors(&state);
Audio audio;

RuntimeTasks runtime(&state, &lightSensor, &leds, &backend, &wifi);

static void performI2CBusScan()
{
  Logger::info("I2C", "Scanning I2C bus...");
  Wire.begin(I2C_SDA, I2C_SCL);

  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      String msg = "Device found at 0x" + String(addr, HEX);
      if (addr == 0x23 || addr == 0x5C)
        msg += " (BH1750)";
      if (addr == 0x3C)
        msg += " (SSD1306 OLED)";
      Logger::info("I2C", msg.c_str());
      found++;
    }
  }

  if (found == 0)
    Logger::error("I2C", "No I2C devices detected!");
}

void setup()
{
  Serial.begin(115200);
  delay(400);
  Logger::info("SYSTEM", "Teletable Firmware Starting...");

  performI2CBusScan();

  display.begin();
  delay(50);

  leds.begin();
  delay(50);

  lightSensor.begin();
  delay(50);

  IR::init();

  motors.begin();
  motors.startTask();

  audio.begin();
  audio.startTask();
  audio.cmdSetVolume(0.4f);
  audio.cmdPlayMelody(true);

  wifi.connect();

  backend.begin();
  api.begin();
  server.begin();

  runtime.begin();
  runtime.start();

  state.lock();
  state.systemHealth = "OK";
  state.unlock();

  Logger::info("SYSTEM", "Setup sequence complete.");
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
