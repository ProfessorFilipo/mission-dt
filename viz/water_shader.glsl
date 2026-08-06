// VERTEX SHADER
#version 130
uniform mat4 p3d_ModelViewProjectionMatrix;
in vec4 p3d_Vertex;
in vec2 p3d_MultiTexCoord0;
out vec2 uv;
out vec3 worldPos;
uniform float time;

void main()
{
    uv = p3d_MultiTexCoord0;
    worldPos = p3d_Vertex.xyz;

    gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;
}

// FRAGMENT SHADER
#version 130
in vec2 uv;
in vec3 worldPos;
out vec4 fragColor;
uniform float time;

void main() {
    // Generate sharp animated normal maps algorithmically
    vec3 normal = normalize(vec3(
        sin(worldPos.x * 4.0 + time * 2.0) * 0.2,
        cos(worldPos.y * 4.0 + time * 2.0) * 0.2,
        1.0
    ));

    // Simulate overhead sun/light source
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.7));
    vec3 viewDir = normalize(cameraPos - worldPos);

    // Diffuse water coloration (Deep blue gradient into lighter turquoise)
    float diffuse = max(dot(normal, lightDir), 0.0);
    vec3 deepWater = vec3(0.005, 0.15, 0.35);
    vec3 shallowWater = vec3(0.1, 0.55, 0.65);
    vec3 baseColor = mix(deepWater, shallowWater, diffuse);

    // Specular Highlight (The bright, realistic sun glare on waves)
    vec3 halfDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfDir), 0.0), 64.0); // 64 = shininess
    vec3 specularColor = vec3(0.9, 0.95, 1.0) * spec * 0.8;

    float fresnel = pow(1.0 - max(dot(normal, viewDir), 0.0), 4.0);

    vec3 skyReflection = vec3(0.55,0.72,0.95) * fresnel * 0.6;

    // fragColor = vec4(baseColor + specularColor, 0.85); // 0.85 opacity for subtle transparency
    vec3 finalColor = baseColor;
    finalColor += specularColor;
    finalColor += skyReflection;

    fragColor = vec4(finalColor,0.90);
}
