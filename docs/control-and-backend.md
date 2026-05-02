# Control And Backend

This firmware exposes a few local and remote control surfaces. The backend side stays intentionally small: the robot reports status, receives commands, and emits alerts.

## HTTP Control Surface

`RobotHttpServer` exposes four endpoints:

- `GET /status`
  Returns a JSON snapshot of robot state, sensors, LED state, and audio volume.
- `POST /mode`
  Accepts a JSON body with `mode` set to `IDLE`, `MANUAL`, or `AUTO`.
- `POST /select`
  Accepts a JSON body with `startNode` and `endNode` to request autonomous navigation.
- `GET /health`
  Returns a simple liveness response.

The HTTP server is local to the robot firmware. It delegates actual behavior back into `BackendCoordinator`, which reads from and updates `RobotState` and `NavigationController`.

Malformed HTTP requests are rejected with JSON errors. Important request problems also produce `FirmwareAlert` warnings.

## Websocket Control Surface

`WsControlClient` is the firmware's remote command receiver. It connects to the backend websocket and forwards parsed commands into callbacks registered by `BackendCoordinator`.

Current command categories handled by the firmware include:

- `NAVIGATE`
- `DRIVE_COMMAND`
- `LED`
- `AUDIO_BEEP`
- `AUDIO_VOLUME`
- `AUDIO_STREAM_START`
- `AUDIO_STREAM_STOP`
- streamed audio binary frames
- `STOP`
- `SET_MODE`

The coordinator decides what each of those commands means for local state and hardware.

Examples:

- `NAVIGATE` requests a graph route
- `DRIVE_COMMAND` switches into manual control if needed and applies drive targets
- `STOP` cancels navigation and returns to `IDLE`
- `SET_MODE` changes the high-level drive mode

Malformed JSON, missing command fields, unknown commands, transport errors, and some rejected commands generate firmware alerts.

## Serial Developer Console

`ConsoleCommander` provides a local developer interface over serial. It is useful for bring-up, calibration, and quick debugging without involving the backend.

Current commands include:

- direct left/right motor commands
- smoothed tank drive commands
- `stop`
- drive debug toggling
- one-shot and periodic sensor prints for IR, lux, IMU, power, and RFID
- LED enable, auto mode, color, and brightness commands
- audio volume and beep commands
- `backendping` for network reachability checks

This console is for operators and developers physically attached to the robot, not for backend automation.

## Backend Relationship

The firmware interacts with the backend through `BackendClient`, `WsControlClient`, and `FirmwareAlert`.

At a high level:

1. The robot registers itself with `/table/register`.
2. The robot periodically pushes condensed state to `/table/state`.
3. The robot listens for remote control commands over websocket.
4. The robot sends important user-facing events to `/table/event`.

The firmware does not contain backend business logic. It only knows how to:

- connect to the configured host and port
- authenticate requests with the robot API key
- serialize state and event payloads
- react to incoming remote commands

## State Reporting

Backend state reporting is coordinated by `BackendCoordinator::stateTask(...)`.

It sends:

- battery level and power data when available
- backend-facing drive mode
- route and target node data
- current localized position
- IMU, RFID, light, and IR status snapshots

State is sent when:

- a relevant local change marks state dirty
- a heartbeat interval expires

This keeps reporting reasonably fresh without posting every loop iteration.

## Alerts

`FirmwareAlert` is the firmware's user-visible event channel. It sends INFO, WARN, and ERROR messages to `/table/event`.

Typical alert sources:

- hardware init failures at boot
- Wi-Fi or websocket connectivity changes
- malformed HTTP or websocket control payloads
- navigation rejections, localization loss, and runtime errors

Alerts are best-effort and use a small duplicate suppression window to avoid flooding the backend with the same message repeatedly.
