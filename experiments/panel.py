"""
Mission-DT control panel -- live terminal dashboard of every agent.

    pip install rich          (optional; falls back to plain text)
    python experiments/panel.py [--host 127.0.0.1]

An independent MQTT client (like the 3D view): subscribes to telemetry
and registration topics and shows, twice per second, one row per agent:
position, altitude, speed, battery, telemetry rate, staleness and
status (OK / STALE / LOWBATT). Runs on any machine that reaches the
broker -- including alongside the simulation and the 3D view.
"""
import argparse
import json
import threading
import time

import paho.mqtt.client as mqtt

STALE_S, LOW_V = 1.5, 17.6


class Watch:
    def __init__(self):
        self.lock = threading.Lock()
        self.a = {}   # aid -> dict

    def reg(self, aid, p):
        with self.lock:
            self.a.setdefault(aid, {}).update(
                domain=p.get("domain", "?"), kind=p.get("kind", "?"))

    def tel(self, aid, p):
        now = time.time()
        with self.lock:
            d = self.a.setdefault(aid, {})
            d.setdefault("count", 0)
            d.setdefault("t0", now)
            d["count"] += 1
            d["t"] = now
            d["lat"], d["lon"], d["alt"] = p["gps"]
            d["spd"] = p["vel"][0]
            d["vb"] = p.get("vb", 0)

    def rows(self):
        now = time.time()
        out = []
        with self.lock:
            for aid in sorted(self.a):
                d = self.a[aid]
                age = now - d.get("t", 0)
                rate = d.get("count", 0) / max(0.001, now - d.get("t0", now))
                st = "STALE" if age > STALE_S else (
                    "LOWBATT" if d.get("vb", 99) < LOW_V else "OK")
                out.append((aid, d.get("domain", "?"), d.get("kind", "?"),
                            f"{d.get('lat', 0):.5f}", f"{d.get('lon', 0):.5f}",
                            f"{d.get('alt', 0):5.1f}", f"{d.get('spd', 0):4.1f}",
                            f"{d.get('vb', 0):5.2f}", f"{rate:4.1f}",
                            f"{age:4.1f}", st))
        return out


HEAD = ("ID", "DOMAIN", "KIND", "LAT", "LON", "ALT m", "SPD", "VBAT",
        "Hz", "AGE s", "STATUS")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    args = ap.parse_args()

    w = Watch()

    def onm(c, u, m):
        parts = m.topic.split("/")
        try:
            p = json.loads(m.payload)
        except json.JSONDecodeError:
            return
        if parts[-1] == "register":
            w.reg(parts[2], p)
        elif parts[-1] == "telemetry":
            w.tel(parts[2], p)

    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="panel")
    cli.on_message = onm
    cli.connect(args.host, 1883)
    cli.subscribe([("missiondt/agents/+/telemetry", 0),
                   ("missiondt/agents/+/register", 1)])
    cli.loop_start()

    try:
        from rich.live import Live
        from rich.table import Table

        def render():
            t = Table(title="Mission-DT agents")
            for h in HEAD:
                t.add_column(h)
            for r in w.rows():
                style = {"OK": "green", "STALE": "grey50",
                         "LOWBATT": "red"}[r[-1]]
                t.add_row(*r, style=style)
            return t

        with Live(render(), refresh_per_second=2) as live:
            while True:
                time.sleep(0.5)
                live.update(render())
    except ImportError:
        print("(rich not installed -- plain mode; pip install rich)")
        while True:
            time.sleep(2)
            print("\n" + " | ".join(HEAD))
            for r in w.rows():
                print(" | ".join(r))


if __name__ == "__main__":
    main()
