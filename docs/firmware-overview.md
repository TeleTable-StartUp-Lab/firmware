# Firmware Overview

This firmware runs an ESP32-based robot with local motor control, sensor polling, a small autonomous navigation layer, an OLED status UI, LED and audio output, and a lightweight backend connection for status, control, and alerts.

The main entrypoints are `App::setup()` and `App::loop()` in `src/app/app.cpp`.

## Boot Sequence

`App::setup()` performs startup in this order:

1. Start serial output and initialize the backend HTTP worker with `BackendClient::begin()`.
2. Initialize the drive controller so the motors start in a known state.
3. Start the serial developer console.
4. Initialize the IR obstacle sensors.
5. Start I2C, set the bus clock, and scan for attached devices.
6. Initialize the OLED, light sensor, power monitor, IMU, RFID reader, LEDs, navigation controller, and audio output.
7. Attempt Wi-Fi connection using credentials from `secrets.h`.
8. Start the backend/control coordinator.
9. If Wi-Fi is available, queue robot registration and the first backend state push.
10. Print a boot summary and emit a `FirmwareAlert` boot event.

Startup failures do not generally abort the whole firmware. Instead, the robot continues to boot and emits warning or error alerts through `FirmwareAlert`.

## Main Loop

`App::loop()` keeps the runtime moving by calling these subsystems in order:

1. `backend.handle()`
   Processes HTTP control requests, websocket traffic, and coordinator drive-mode housekeeping.
2. `statusPrintTask(nowMs)`
   Prints periodic serial status snapshots for debugging.
3. `sensors.update(nowMs)`
   Polls IR, light, power, IMU, and RFID sensors.
4. `audio.loop()`
   Advances audio playback and streaming.
5. `navigation.update(nowMs)`
   Advances autonomous navigation, including RFID localization and turn completion logic.
6. `oled.update(nowMs)`
   Refreshes the on-device display.
7. `leds.autoTask()`
   Updates automatic LED behavior.
8. `console.handle()`
   Processes developer commands from serial input.
9. `drive.update(nowMs, state.driveMode())`
   Applies smoothed motor commands and drive safety rules.
10. `backend.registerTask(nowMs)` and `backend.stateTask(nowMs)`
    Keep backend registration alive and push state changes or heartbeat updates.

The loop ends with a short `delay(1)` to avoid a tight spin.

## Main Subsystems

### `SensorSuite`

`SensorSuite` wraps the robot sensors behind one interface:

- IR obstacle sensors
- BH1750 light sensor
- INA226 power monitor
- MPU6050 IMU
- RC522 RFID reader

It owns initialization, periodic updates, one-shot debug printing, and simple accessors used elsewhere in the firmware.

### `DriveController`

`DriveController` converts throttle and steer targets into left and right motor outputs. It handles:

- input clamping
- smoothing and slew limiting
- deadzone and expo shaping
- tank mixing
- optional debug output

There is placeholder logic for IR-based forward blocking, but obstacle enforcement is currently disabled in code with `TODO` guards. Documented behavior should reflect that the structure exists, but autonomous and manual drive are not currently using live IR braking.

### `NavigationController`

`NavigationController` provides graph-based autonomous routing between logical nodes such as `home`, `kitchen`, `office`, and `grave`.

It is responsible for:

- mapping node IDs to RFID-backed locations
- planning a path over the hard-coded graph
- converting graph edges into actions such as straight, reverse, turn left, and turn right
- updating robot state during navigation
- recovering or rejecting requests when localization is lost

See `docs/navigation-graph.md` for the detailed model and extension guide.

### `RobotState`

`RobotState` is the shared runtime snapshot for higher-level behavior. It stores:

- current drive mode
- last requested route
- current position/current node
- target node
- navigation status

This state is what the backend coordinator exposes through HTTP and backend status messages.

### `BackendCoordinator`

`BackendCoordinator` connects firmware behavior to network-facing control and reporting. It:

- starts the local HTTP server
- configures websocket control callbacks
- requests navigation
- updates drive mode in response to control commands
- pushes state to the backend
- periodically re-registers the robot

It is the main bridge between local robot behavior and remote control.

### `OledUi`

`OledUi` renders local status, including mode and connectivity information, onto the display. It is a local observability layer rather than a control path.

### `FirmwareAlert`

`FirmwareAlert` is the user-facing alert path for important firmware events. It sends INFO/WARN/ERROR events through the backend event endpoint and lightly suppresses repeated duplicate alerts.

Current alert sources include:

- startup hardware failures
- Wi-Fi and websocket connectivity changes
- malformed control requests
- navigation errors and important route/lifecycle events

## Drive Modes and State Flow

The firmware uses three drive modes from `RobotHttpServer::DriveMode`:

- `IDLE`: no active autonomous route and no live manual control
- `MANUAL`: remote direct drive commands are controlling the robot
- `AUTO`: autonomous navigation is active or has been selected

`RobotState::driveModeToBackend()` converts internal drive state into backend-facing values:

- `MANUAL` stays `MANUAL`
- `AUTO` becomes `NAVIGATING` only while navigation status is `PLANNING`, `TURNING`, or `DRIVING`
- everything else becomes `IDLE`

That means the backend view is intentionally simpler than the internal firmware state machine.

## Backend Interaction

The firmware/backend relationship is intentionally small:

- `BackendClient` registers the robot with `/table/register`
- `BackendCoordinator` queues status snapshots to `/table/state`
- `WsControlClient` receives websocket commands for navigation, manual drive, LEDs, audio, stop, and mode switching
- `FirmwareAlert` sends important events to `/table/event`

The firmware does not need deep backend knowledge to operate. It only needs the configured host, port, transport settings, and API key.

## Current Limitations

- The autonomous map is hard-coded in `NavigationController`.
- RFID tags are the only localization source used for graph-node confirmation.
- Turn logic assumes cardinal-style headings and 90-degree maneuvers.
- IR obstacle detection exists conceptually in drive and navigation, but it is currently disabled in both places.
- Larger environments will eventually stress the fixed-size compile-time graph arrays and `MAX_PATH_STEPS`.
