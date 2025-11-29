#version 450

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color; // we use this as normal (r,g,b)

layout(location = 0) out vec3 vNormal;    // world-space normal
layout(location = 1) out vec3 vWorldPos;  // world-space position
layout(location = 2) out vec3 vColor;     // base color (optional)

layout(push_constant) uniform mod {
    mat4 model;
} model;

layout(set = 0, binding = 0) uniform cam {
    mat4 view;
    mat4 projection;
} camera;

out gl_PerVertex { vec4 gl_Position; };

void main()
{
    // world-space position
    vec4 worldPos4 = model.model * vec4(position, 1.0);
    vWorldPos = worldPos4.xyz;

    // compute normal matrix (inverse-transpose of model's upper-left 3x3)
    mat3 normalMat = transpose(inverse(mat3(model.model)));
    vec3 n = normalize(normalMat * color); // color holds normal (r,g,b)
    vNormal = n;

    // pass a gray base color so fragment shader has something if needed
    vColor = vec3(0.7, 0.65, 0.55);

    gl_Position = camera.projection * camera.view * worldPos4;
}
