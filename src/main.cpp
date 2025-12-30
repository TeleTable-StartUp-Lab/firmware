#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <FastLED.h>
#include <Wire.h>

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
#include "pins.h"

// Global instances
AsyncWebServer server(80);
SystemState state;
WifiManager wifi;
ApiHandler api(&server, &state);

// Hardware Controllers
DisplayController display;
LEDController leds;
BH1750Sensor lightSensor;

/**
 * Scans the I2C bus for connected devices to ensure hardware is present.
 */
void performI2CBusScan()
{
  Logger::info("I2C", "Scanning I2C bus...");
  Wire.begin(I2C_SDA, I2C_SCL);
  int devicesFound = 0;

  for (byte address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0)
    {
      String msg = "Device found at 0x" + String(address, HEX);
      if (address == 0x23 || address == 0x5C)
        msg += " (BH1750)";
      if (address == 0x3C)
        msg += " (SSD1306 OLED)";
      Logger::info("I2C", msg.c_str());
      devicesFound++;
    }
  }
  if (devicesFound == 0)
    Logger::error("I2C", "No I2C devices detected!");
}

/**
 * Controls lighting based on ambient light intensity (Lux).
 */
void handleAutomaticLighting()
{
  // Hysteresis logic to prevent flickering
  const float LUX_ON_THRESHOLD = 25.0f;
  const float LUX_OFF_THRESHOLD = 40.0f;
  static bool lightsActive = false;

  if (!lightsActive && state.lux < LUX_ON_THRESHOLD && state.lux >= 0)
  {
    Logger::info("LIGHTS", "Low light detected. Activating LEDs.");
    leds.setAllColors(CRGB(200, 150, 50)); // Warm White
    FastLED.show();
    lightsActive = true;
  }
  else if (lightsActive && state.lux > LUX_OFF_THRESHOLD)
  {
    Logger::info("LIGHTS", "Sufficient light detected. Deactivating LEDs.");
    leds.clear();
    FastLED.show();
    lightsActive = false;
  }
}

void setup()
{
  // 1. Core Initialization
  Serial.begin(115200);
  delay(2500); // Power stability delay
  Logger::info("SYSTEM", "Teletable Firmware Starting...");

  // 2. Hardware Setup (Staggered to prevent current spikes)
  performI2CBusScan();

  display.begin();
  delay(100);

  leds.begin();
  delay(100);

  lightSensor.begin();
  delay(100);

  IR::init();

  // 3. Network and API Setup
  wifi.connect();
  api.begin();
  server.begin();

  state.systemHealth = "OK";
  Logger::info("SYSTEM", "Setup sequence complete.");
}

void loop()
{
  unsigned long currentMillis = millis();
  static unsigned long lastSensorUpdate = 0;
  static unsigned long lastWifiCheck = 0;

  // 1. Sensor and Logic Update Task (500ms)
  if (currentMillis - lastSensorUpdate >= 500)
  {
    // Read hardware values into global state
    state.lux = lightSensor.readLight();
    state.obstacleLeft = (IR::readLeft() == 0);
    state.obstacleRight = (IR::readRight() == 0);

    // Debug output for monitoring
    String sensorData = "Lux: " + String(state.lux) +
                        " | L-Obs: " + String(state.obstacleLeft ? "YES" : "NO") +
                        " | R-Obs: " + String(state.obstacleRight ? "YES" : "NO");
    Logger::debug("SENSOR", sensorData.c_str());

    // Process autonomous behavior
    handleAutomaticLighting();

    lastSensorUpdate = currentMillis;
  }

  // 2. Connectivity Watchdog Task (10s)
  if (currentMillis - lastWifiCheck >= 10000)
  {
    if (!wifi.isConnected())
    {
      Logger::warn("SYSTEM", "Connectivity lost.");
      state.systemHealth = "WiFi Error";
    }
    else
    {
      state.systemHealth = "OK";
    }
    lastWifiCheck = currentMillis;
  }
}