#version 450

layout(location = 0) in vec3 vNormal;
layout(location = 1) in vec3 vWorldPos;
layout(location = 2) in vec3 vColor;

layout(location = 0) out vec4 fragColor;

layout(set = 0, binding = 0) uniform cam {
    mat4 view;
    mat4 projection;
} camera;

void main()
{
    // Simple directional light (world-space)
    vec3 lightDir = normalize(vec3(0.5, -1.0, 0.3)); // tweak this to change light direction
    vec3 lightColor = vec3(1.0, 0.98, 0.9);

    // Material
    vec3 baseColor = vColor; // could also be computed from elevation if you want
    float ambientStrength = 0.2;
    float shininess = 32.0;
    float specularStrength = 0.5;

    // Normal (already normalized in VS but normalize again to be safe)
    vec3 N = normalize(vNormal);

    // Diffuse
    float diff = max(dot(N, -lightDir), 0.0); // lightDir points from light to surface -> negate if needed
    vec3 diffuse = diff * lightColor;

    // View direction: compute camera position in world space
    // inverse(camera.view) transforms from view to world. camera.view is world->view so invert it.
    vec3 viewPosWorld = (inverse(camera.view) * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
    vec3 V = normalize(viewPosWorld - vWorldPos);

    // Blinn-Phong specular
    vec3 H = normalize(V - lightDir); // half vector between view and light (lightDir is from light to surface)
    float spec = pow(max(dot(N, H), 0.0), shininess);
    vec3 specular = specularStrength * spec * lightColor;

    // Ambient term
    vec3 ambient = ambientStrength * lightColor;

    vec3 result = (ambient + diffuse + specular) * baseColor;

    fragColor = vec4(result, 1.0);
}
