import sys
import time
import socket
import threading
import json
import logging
import os
from datetime import datetime, timezone
# Install requirements if missing (mock check, assume user handles it or run pip install -r requirements_robot.txt)

try:
    import websocket
    from flask import Flask, jsonify, request
    import requests
except ImportError:
    print("Missing dependencies. Please run: pip install -r requirements_robot.txt")
    sys.exit(1)

# Configuration
# Allow overriding via environment variables
BACKEND_HOST = os.environ.get("BACKEND_HOST", "localhost")
BACKEND_PORT = os.environ.get("BACKEND_PORT", "3003")
BACKEND_UDP_PORT = int(os.environ.get("BACKEND_UDP_PORT", 3001))

BACKEND_HTTP_URL = f"http://{BACKEND_HOST}:{BACKEND_PORT}"
BACKEND_WS_URL = f"ws://{BACKEND_HOST}:{BACKEND_PORT}/ws/robot/control"
MY_PORT = int(os.environ.get("ROBOT_PORT", 8000))
API_KEY = os.environ.get("ROBOT_API_KEY", "secret-robot-key")

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

app = Flask(__name__)

# Mock Robot State
robot_state = {
    "systemHealth": "OK",
    "batteryLevel": 50,
    "driveMode": "IDLE",
    "cargoStatus": "EMPTY",
    "currentPosition": "Home",
    "lastNode": None,
    "targetNode": None,
}


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok", "message": "Robot is online"})


@app.route("/status", methods=["GET"])
def get_status():
    return jsonify(robot_state)


@app.route("/nodes", methods=["GET"])
def get_nodes():
    return jsonify(
        {
            "nodes": [
                "Home",
                "Kitchen",
                "Living Room",
                "Office",
                "Bedroom",
                "Charging Station",
            ]
        }
    )


def run_flask():
    logger.info(f"Starting Robot Web Server on port {MY_PORT}")
    app.run(host="0.0.0.0", port=MY_PORT, debug=False, use_reloader=False)


def send_udp_broadcast():
    logger.info(f"Sending UDP broadcast to backend port {BACKEND_UDP_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    message = json.dumps({"type": "announce", "port": MY_PORT})

    try:
        sock.sendto(message.encode("utf-8"), ("<broadcast>", BACKEND_UDP_PORT))
        logger.info("UDP Broadcast sent.")
    except Exception as e:
        logger.error(f"Failed to send UDP broadcast: {e}")
    finally:
        sock.close()


def send_client_event(event):
    now = datetime.now(timezone.utc)

    robot_event = {"event": event, "timestamp": now.strftime("%Y-%m-%dT%H:%M:%SZ")}

    headers = {"X-Api-Key": API_KEY, "Content-Type": "application/json"}
    try:
        logger.info("Sending event to backend...")
        resp = requests.post(
            f"{BACKEND_HTTP_URL}/table/event", json=robot_event, headers=headers
        )
        if resp.status_code == 200:
            logger.info("Successfully sent event backend")
        else:
            logger.warning(f"Failed sent event: {resp.status_code} - {resp.text}")
    except Exception as e:
        logger.error(f"Exception sending event: {e}")


def on_message(ws, message):
    logger.info(f"Received WS message: {message}")
    try:
        cmd = json.loads(message)
        command_type = cmd.get("command")

        if command_type == "NAVIGATE":
            send_client_event("START_BUTTON_PRESSED")
            start = cmd.get("start")
            dest = cmd.get("destination")
            logger.info(f"Navigating from {start} to {dest}")

            # Simulate state change
            robot_state["driveMode"] = "NAVIGATING"
            robot_state["currentPosition"] = "MOVING"
            robot_state["targetNode"] = dest
            robot_state["lastNode"] = start
            update_backend_state()

            # Simulate time passing for movement
            def finish_move():
                time.sleep(5)
                logger.info(f"Arrived at {dest}")
                robot_state["currentPosition"] = dest
                robot_state["driveMode"] = "IDLE"
                update_backend_state()
                send_client_event("DESTINATION_REACHED")

            # Run simulation in thread so we don't block WS
            threading.Thread(target=finish_move).start()

        elif command_type == "SET_MODE":
            mode = cmd.get("mode")
            logger.info(f"Setting mode to {mode}")
            robot_state["driveMode"] = mode
            update_backend_state()

        elif command_type == "DRIVE_COMMAND":
            linear = cmd.get("linear_velocity", 0.0)
            angular = cmd.get("angular_velocity", 0.0)
            logger.info(f"Manual Drive: Linear={linear}, Angular={angular}")
            # In a real robot, this would move motors.
            # Here we just log it and maybe update state to 'MANUAL' if not already?
            if robot_state["driveMode"] != "MANUAL":
                robot_state["driveMode"] = "MANUAL"
                update_backend_state()

    except Exception as e:
        logger.error(f"Error parsing message: {e}")


def on_error(ws, error):
    logger.error(f"WebSocket Error: {error}")


def on_close(ws, close_status_code, close_msg):
    logger.info("WebSocket Closed")


def on_open(ws):
    logger.info("WebSocket Opened")
    # Send initial state
    update_backend_state()


def update_backend_state():
    headers = {"X-Api-Key": API_KEY, "Content-Type": "application/json"}
    try:
        logger.info("Sending state update to backend...")
        resp = requests.post(
            f"{BACKEND_HTTP_URL}/table/state", json=robot_state, headers=headers
        )
        if resp.status_code == 200:
            logger.info("Successfully updated state to backend")
        else:
            logger.warning(f"Failed to update state: {resp.status_code} - {resp.text}")
    except Exception as e:
        logger.error(f"Exception updating state: {e}")


def run_websocket():
    # Retry loop
    while True:
        try:
            logger.info(f"Connecting to WebSocket: {BACKEND_WS_URL}")
            ws = websocket.WebSocketApp(
                BACKEND_WS_URL,
                on_open=on_open,
                on_message=on_message,
                on_error=on_error,
                on_close=on_close,
            )
            ws.run_forever()
        except Exception as e:
            logger.error(f"WebSocket connection failed: {e}")

        logger.info("Reconnecting in 5 seconds...")
        time.sleep(5)


if __name__ == "__main__":
    # Start Flask Server in background thread
    t_flask = threading.Thread(target=run_flask)
    t_flask.daemon = True
    t_flask.start()

    # Wait for server to start
    time.sleep(1)

    # Send UDP broadcast periodically or just once?
    # Usually once on startup is enough if backend is running.
    # But if backend restarts, it loses the IP.
    # Let's send it periodically in a separate thread, just to be robust for testing.

    def announce_loop():
        while True:
            send_udp_broadcast()
            time.sleep(10)  # Announce every 10 seconds

    t_announce = threading.Thread(target=announce_loop)
    t_announce.daemon = True
    t_announce.start()

    # Start WebSocket Client (main thread)
    run_websocket()
