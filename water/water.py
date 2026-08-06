
import os
import math
from ursina import Entity, Grid, Shader, color


class Water:
    """Animated water surface with shader and fallback procedural deformation."""

    def __init__(self, scale=300, grid_size=80, use_shader=True):
        """
        Create an animated water surface.

        Args:
            scale: Size of the water mesh (300-600 recommended for mission viz)
            grid_size: Grid resolution (80-120 recommended for smooth waves)
            use_shader: Try to load and use GLSL shader
        """
        self.scale = scale
        self.grid_size = grid_size
        self.shader = None
        self.entity = None
        self.mesh = None
        self.original_verts = []

        # Try to load shader
        if use_shader:
            self._load_shader()

        # Create mesh
        self._create_mesh()

    def _load_shader(self):
        """Load water shader from file."""
        shader_path = os.path.join(
            os.path.dirname(__file__), 'water_shader.glsl'
        )

        try:
            with open(shader_path, 'r') as f:
                shader_code = f.read()

            parts = shader_code.split('// FRAGMENT SHADER')
            self.shader = Shader(
                language=Shader.GLSL,
                vertex=parts[0],
                fragment=parts[1]
            )
            print("[Water] Shader loaded successfully")
        except Exception as e:
            print(f"[Water] Shader load failed: {e}")
            self.shader = None

    def _create_mesh(self):
        """Create the water mesh entity."""
        # Use higher resolution grid for visible grid pattern
        self.mesh = Grid(120, 120)

        # Store original vertices for animation
        if hasattr(self.mesh, 'vertices'):
            self.original_verts = list(self.mesh.vertices)

        # Create entity with texture for reflections/shades
        self.entity = Entity(
            model=self.mesh,
            scale=600,
            rotation_x=-90,
            shader=None,
            color=color.rgb(0.05, 0.6, 0.85),
            texture="white_cube",
            texture_scale=(100, 100)  # Grid pattern
        )

        self.entity.always_on_top = False

        self.entity.y = -0.2  # Close to agents' feet
        self.entity.z = 0
        self.entity.x = 0

        print(f"[Water] Grid (120x120, scale 600) with texture at y=-48")

    def update(self, current_time):

        # Create dynamic shading by varying color based on time
        # Simulate light reflections and depth variations
        base_r = 0.05
        base_g = 0.6
        base_b = 0.85

        # Add reflection shimmer
        shimmer = math.sin(current_time * 2.0) * 0.1

        # Add depth shading - darker edges
        shade_variation = math.cos(current_time * 0.8) * 0.05

        adjusted_color = color.rgb(
            max(0.0, base_r + shimmer * 0.05 + shade_variation),
            max(0.0, base_g + shimmer * 0.1 + shade_variation),
            max(0.0, base_b + shimmer * 0.08 + shade_variation * 0.5)
        )
        self.entity.color = adjusted_color
