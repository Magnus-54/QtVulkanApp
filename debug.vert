#version 450

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

layout(location = 0) out vec3 vNormal;

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
    mat3 normalMat = transpose(inverse(mat3(model.model)));
    vNormal = normalize(normalMat * normal);

    vec4 worldPos = model.model * vec4(position, 1.0);
    gl_Position = camera.projection * camera.view * worldPos;
}
