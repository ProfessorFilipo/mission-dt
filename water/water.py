import math
import os

from ursina import Entity, Grid, Shader, camera, color


class Water:
    def __init__(self, scale=300, grid_size=80, use_shader=False):

        self.scale = scale
        self.grid_size = grid_size
        self.entity = None
        self.grid_entity = None
        self.shader = None
        self._shader_enabled = bool(use_shader)

        if self._shader_enabled:
            self._load_shader()

        # Create mesh
        self._create_mesh()

    def _load_shader(self):
        shader_path = os.path.join(os.path.dirname(__file__), "water_shader.glsl")
        try:
            with open(shader_path, "r", encoding="utf-8") as f:
                shader_code = f.read()
            parts = shader_code.split("// FRAGMENT SHADER", 1)
            if len(parts) != 2:
                self.shader = None
                self._shader_enabled = False
                print("[Water] Shader format invalid; using fallback water.")
                return
            self.shader = Shader(
                language=Shader.GLSL,
                vertex=parts[0],
                fragment=parts[1],
            )
            print("[Water] Shader loaded successfully")
        except Exception as exc:
            self.shader = None
            self._shader_enabled = False
            print(f"[Water] Shader load failed: {exc}")

    def _create_mesh(self):
        # Main water surface (receives shader reflections)
        self.entity = Entity(
            model="plane",
            scale=self.scale,
            color=color.rgb(0.18, 0.35, 0.50),
            shader=self.shader if self._shader_enabled else None,
        )

        # Grid overlay on top of water
        self.grid_entity = Entity(
            model=Grid(self.grid_size, self.grid_size),
            scale=self.scale,
            rotation_x=-90,
            color=color.rgb(0.30, 0.30, 0.30),
            unshaded=True,
        )

        self.entity.y = -0.2
        self.entity.z = 0
        self.entity.x = 0
        self.grid_entity.y = -0.195
        self.grid_entity.z = 0
        self.grid_entity.x = 0

        print(
            f"[Water] Water+grid ({self.grid_size}x{self.grid_size}, "
            f"scale {self.scale})"
        )

    def update(self, current_time):
        if not self.entity:
            return

        if self._shader_enabled and self.shader is not None:
            self.entity.set_shader_input("time", current_time)
            self.entity.set_shader_input("camera_pos", camera.world_position)
        else:
            shimmer = math.sin(current_time * 1.5) * 0.03
            self.entity.color = color.rgb(
                max(0.10, 0.18 + shimmer),
                max(0.20, 0.35 + shimmer * 0.6),
                max(0.32, 0.50 + shimmer * 0.5),
            )
