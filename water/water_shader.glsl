// VERTEX SHADER
#version 120

uniform mat4 p3d_ModelViewProjectionMatrix;
uniform mat4 p3d_ModelMatrix;
attribute vec4 p3d_Vertex;

varying vec3 worldPos;

void main() {
    vec4 position = p3d_ModelMatrix * p3d_Vertex;
    worldPos = position.xyz;
    gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;
}

// FRAGMENT SHADER
#version 120

varying vec3 worldPos;

uniform float time;
uniform vec3 camera_pos;

// Pre-normalized to keep the shader compatible with GLSL 1.30 compilers.
const vec3 SUN_DIRECTION = vec3(-0.35, 0.72, 0.60);
const vec3 SUN_COLOR = vec3(1.0, 0.92, 0.75);

// Two overlapping wave trains produce a subtle, constantly moving surface.
float waveHeight(vec2 p) {
    return sin(p.x * 0.095 + time * 0.75) * 0.34
         + sin(p.y * 0.130 - time * 0.54) * 0.22
         + sin((p.x + p.y) * 0.058 + time * 0.32) * 0.44;
}

vec3 waterNormal(vec2 p) {
    float dhdx = cos(p.x * 0.095 + time * 0.75) * 0.095 * 0.34
               + cos((p.x + p.y) * 0.058 + time * 0.32) * 0.058 * 0.44;
    float dhdz = cos(p.y * 0.130 - time * 0.54) * 0.130 * 0.22
               + cos((p.x + p.y) * 0.058 + time * 0.32) * 0.058 * 0.44;
    return normalize(vec3(-dhdx, 1.0, -dhdz));
}

void main() {
    vec2 wavePoint = worldPos.xz;
    vec3 normal = waterNormal(wavePoint);
    vec3 viewDir = normalize(camera_pos - worldPos);

    // Fresnel makes shallow viewing angles reflective, like real water.
    float facing = clamp(dot(normal, viewDir), 0.0, 1.0);
    float fresnel = 0.04 + 0.96 * pow(1.0 - facing, 5.0);

    vec3 reflectedDir = reflect(-viewDir, normal);
    float skyAmount = clamp(reflectedDir.y * 0.5 + 0.5, 0.0, 1.0);
    vec3 horizonSky = vec3(0.10, 0.26, 0.40);
    vec3 zenithSky = vec3(0.48, 0.71, 0.88);
    vec3 reflection = mix(horizonSky, zenithSky, skyAmount);

    // A restrained glint adds motion without overpowering the scene.
    vec3 halfDir = normalize(SUN_DIRECTION + viewDir);
    float sunGlint = pow(max(dot(normal, halfDir), 0.0), 180.0);
    float ripples = 0.75 + 0.25 * sin(waveHeight(wavePoint) * 14.0 + time * 2.0);

    vec3 deepWater = vec3(0.008, 0.075, 0.16);
    vec3 waterColor = mix(deepWater, vec3(0.025, 0.19, 0.31), facing);
    vec3 color = mix(waterColor, reflection, fresnel * 0.78);
    color += SUN_COLOR * sunGlint * ripples * 0.30;

    gl_FragColor = vec4(color, 1.0);
}
