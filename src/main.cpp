#include <Arduino.h>
#include <ESPAsyncWebServer.h>

// Core and Network
#include "core/system_state.h"
#include "network/wifi_manager.h"
#include "network/api_handler.h"

// Hardware and Utils
#include "hardware/display.h"
#include "hardware/leds.h"
#include "hardware/bh1750.h"
#include "hardware/ir_sensors.h"
#include "utils/logger.h"

// Global instances
AsyncWebServer server(80);
SystemState state; // Global state object
WifiManager wifi;
ApiHandler api(&server, &state);

// Hardware Controllers
DisplayController display;
LEDController leds;
BH1750Sensor lightSensor;

void setup()
{
  // 1. Initialize Serial Communication
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(1000);

  Logger::info("SYSTEM", "Teletable Firmware starting...");

  // 2. Initialize Hardware Components
  Logger::info("SYSTEM", "Initializing hardware...");

  // Note: These might fail if hardware is missing, but the firmware continues
  display.begin();
  leds.begin();
  lightSensor.begin();
  IR::init();

  // 3. Network Setup
  // This will block until WiFi is connected (as per your current wifi_manager.cpp)
  wifi.connect();

  // Set up API routes and start the web server
  api.begin();
  server.begin();

  Logger::info("SYSTEM", "Network services active");

  // 4. Finalize System State
  state.systemHealth = "OK";
  Logger::info("SYSTEM", "Setup sequence finished successfully");
}

void loop()
{
  // A. Keep track of timing for non-blocking execution
  static unsigned long lastSensorUpdate = 0;
  unsigned long currentMillis = millis();

  // B. Periodic Sensor Updates)
  if (currentMillis - lastSensorUpdate >= 500)
  {
    state.lux = lightSensor.readLight();
    state.obstacleLeft = (IR::readLeft() == 0);
    state.obstacleRight = (IR::readRight() == 0);
    // Here we could also update battery level
    lastSensorUpdate = currentMillis;
  }

  // C. Connection Watchdog
  static unsigned long lastWifiCheck = 0;
  if (currentMillis - lastWifiCheck >= 10000)
  {
    if (!wifi.isConnected())
    {
      Logger::warn("SYSTEM", "WiFi connection lost. State: DISCONNECTED");
      state.systemHealth = "WiFi Error";
    }
    else
    {
      state.systemHealth = "OK";
    }
    lastWifiCheck = currentMillis;
  }

  // D. Background tasks (e.g., LED animations) can be added here
}