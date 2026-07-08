#!/bin/bash
# Mission-DT -- launch a full mission (broker + core + 3D view) in one go.
#
#   ./run_mission.sh                                # default mission
#   ./run_mission.sh configs/mission_photo.json     # any mission file
#
# Checks every dependency and starts what is missing:
#   venv -> file-descriptor limit -> MQTT broker -> retained cleanup
#   -> mission core (background) -> 3D view (foreground).
# Closing the 3D view (ESC) also stops the mission core.

set -e
cd "$(dirname "$0")"
CFG="${1:-configs/mission_default.json}"

if [ ! -f "$CFG" ]; then
    echo "ERROR: mission config not found: $CFG"
    echo "Available missions:"
    ls -1 configs/*.json 2>/dev/null | sed 's/^/  /'
    exit 1
fi

# --- 1) virtual environment (idempotent) ------------------------------
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
else
    echo "ERROR: .venv not found. Create it first:  python3 -m venv .venv"
    exit 1
fi
ulimit -n 4096 2>/dev/null || true

# --- 2) MQTT broker: check, and start it if needed --------------------
broker_up () {
python - <<'PY' 2>/dev/null
import socket; socket.create_connection(("127.0.0.1", 1883), 2).close()
PY
}

if ! broker_up; then
    echo "Broker not running -- starting Mosquitto..."
    if command -v brew >/dev/null 2>&1; then
        brew services start mosquitto >/dev/null 2>&1 || true
        sleep 2
    fi
    if ! broker_up && command -v mosquitto >/dev/null 2>&1; then
        # fallback: run the binary in the background (log in /tmp)
        nohup mosquitto > /tmp/mosquitto_mission.log 2>&1 &
        sleep 2
    fi
    if ! broker_up; then
        echo "ERROR: could not start an MQTT broker on 127.0.0.1:1883."
        echo "  macOS:  brew install mosquitto     Linux:  apt install mosquitto"
        exit 1
    fi
fi
echo "Broker OK."

# --- 3) clean retained ghosts (old registrations, checkpoints, routes)
python experiments/clear_retained.py

# --- 4) mission core in the background --------------------------------
echo "Launching mission: $CFG"
python experiments/demo_mission.py --config "$CFG" &
DEMO_PID=$!
trap 'echo "Stopping mission core..."; kill $DEMO_PID 2>/dev/null || true' EXIT
sleep 1
if ! kill -0 $DEMO_PID 2>/dev/null; then
    echo "ERROR: mission core exited early -- check the config file."
    exit 1
fi

# --- 5) 3D view in the foreground (ESC quits everything) --------------
echo "Opening the 3D mission view (ESC to quit)."
echo "Tip: for the live agent table, run in another terminal:"
echo "     source .venv/bin/activate && python experiments/panel.py"
python viz/mission_viz.py
