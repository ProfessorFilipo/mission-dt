#!/bin/bash
# Mission-DT -- run the full experiment battery (E1..E4 + figures).
# Usage, from anywhere:   bash run_all.sh
# Safe both inside and outside PyCharm: venv activation is idempotent.

set -e
cd "$(dirname "$0")"                      # repo root = script location

# 1) virtual environment (re-activating an active venv is harmless)
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
else
    echo "ERROR: .venv not found. Create it first:  python3 -m venv .venv"
    exit 1
fi

# 2) macOS default file-descriptor limit (256) is too low for 100 agents
ulimit -n 4096 2>/dev/null || true

# 3) broker sanity check
if ! python - <<'PY' 2>/dev/null
import socket; socket.create_connection(("127.0.0.1", 1883), 2).close()
PY
then
    echo "ERROR: no MQTT broker on 127.0.0.1:1883."
    echo "  macOS:  brew services start mosquitto   (or run: mosquitto -v)"
    echo "  Linux:  sudo systemctl start mosquitto"
    exit 1
fi

echo "== Clearing retained ghosts from the broker =="
python experiments/clear_retained.py

echo "== E1 + E2 (scalability + regulators, ~6 min) =="
python experiments/run_experiments.py

echo "== E3 (swarm propagation, ~2.5 min) =="
python experiments/run_e3.py

echo "== E4 (twin fidelity vs. packet loss, ~2 min) =="
python experiments/run_e4.py

echo "== Figures =="
python experiments/make_figures.py

echo "All experiments done. Raw data and figures are in results/."
