"""
Mission-DT 3D visualizer (run LOCALLY on a machine with a GPU/display).

    pip install ursina paho-mqtt
    python viz/mission_viz.py --host <broker-ip>

Subscribes to missiondt/agents/+/telemetry and renders every hybrid
agent (physical or virtual, aerial or surface) in a shared 3D scene at
interactive frame rates. Aerial agents render as cones, surface agents
as boxes; a trail shows the recent trajectory (the [B^i;B^j] window).
Press F to print the rendering FPS (report this number in the paper).
"""
import argparse
import json
import math
import threading

import paho.mqtt.client as mqtt
from ursina import (Ursina, Entity, color, camera, window, Text,
                    application, held_keys, time as utime)

BASE_LAT, BASE_LON = -30.0577, -51.1729
SCALE = 111_320.0 / 10.0     # 1 scene unit = 10 m


class Store:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = {}          # aid -> (x, y, z, yaw, domain)

    def update(self, aid, p):
        lat, lon, alt = p["gps"]
        x = (lon - BASE_LON) * SCALE * math.cos(math.radians(lat))
        z = (lat - BASE_LAT) * SCALE
        dom = "aerial" if alt > 1.0 else "surface"
        with self.lock:
            self.state[aid] = (x, alt / 10.0, z, p["att"][2], dom)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    args = ap.parse_args()

    store = Store()
    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="viz")
    cli.on_message = lambda c, u, m: store.update(
        m.topic.split("/")[2], json.loads(m.payload))
    cli.connect(args.host, 1883)
    cli.subscribe("missiondt/agents/+/telemetry")
    cli.loop_start()

    app = Ursina(title="Mission-DT")
    window.color = color.rgb(10, 20, 40)
    Entity(model="plane", scale=400, color=color.rgb(20, 60, 90),
           texture="white_cube", texture_scale=(80, 80))
    camera.position = (0, 60, -80)
    camera.rotation_x = 35
    hud = Text(text="", position=(-0.85, 0.47), scale=0.8)
    ents, trails = {}, {}

    def update():
        with store.lock:
            snap = dict(store.state)
        for aid, (x, y, z, yaw, dom) in snap.items():
            if aid not in ents:
                if dom == "aerial":
                    e = Entity(model="cone", color=color.orange, scale=(1.5, 2.5, 1.5))
                else:
                    e = Entity(model="cube", color=color.azure, scale=(1.2, 0.6, 3.0))
                ents[aid] = e
                trails[aid] = []
            e = ents[aid]
            e.position = (x, max(0.3, y), z)
            e.rotation_y = math.degrees(yaw)
            trails[aid].append(Entity(model="sphere", scale=0.15,
                                      color=color.light_gray, position=e.position))
            if len(trails[aid]) > 60:
                trails[aid].pop(0).disabled = True
        hud.text = f"agents: {len(ents)}   FPS: {int(1/max(utime.dt,1e-6))}"
        if held_keys["escape"]:
            application.quit()

    app.update = update  # noqa
    Entity(update=update)
    app.run()


if __name__ == "__main__":
    main()
