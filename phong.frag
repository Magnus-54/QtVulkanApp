#version 450

layout(location = 0) in vec3 vNormal;
layout(location = 0) out vec4 fragColor;

void main()
{
    // Clamp and transform normals from [-1,1] to [0,1]
    vec3 N = normalize(vNormal);
    vec3 c = N * 0.5 + 0.5;

    fragColor = vec4(c, 1.0);
}
