"""
Clear every retained Mission-DT message from the broker (ghost agent
registrations, stale checkpoints, old routes). Run before experiments
or demos, or let run_all.sh call it automatically.

    python experiments/clear_retained.py [--host 127.0.0.1]
"""
import argparse
import threading
import time

import paho.mqtt.client as mqtt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    args = ap.parse_args()

    topics, lock = set(), threading.Lock()

    def onm(c, u, m):
        if m.retain and m.payload:
            with lock:
                topics.add(m.topic)

    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="janitor")
    cli.on_message = onm
    cli.connect(args.host, 1883)
    cli.subscribe("missiondt/#", qos=1)
    cli.loop_start()
    time.sleep(1.5)                      # collect retained backlog
    with lock:
        found = sorted(topics)
    for t in found:                      # empty retained payload = delete
        cli.publish(t, payload=b"", qos=1, retain=True)
    time.sleep(0.5)
    cli.loop_stop()
    cli.disconnect()
    print(f"cleared {len(found)} retained topic(s)"
          + (":" if found else ""))
    for t in found[:20]:
        print("  ", t)
    if len(found) > 20:
        print(f"   ... and {len(found) - 20} more")


if __name__ == "__main__":
    main()
