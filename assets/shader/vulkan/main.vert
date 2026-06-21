#version 450
layout(std140, binding = 0) uniform vs_params {
    mat4 model;
    mat4 view;
    mat4 projection;
};
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec4 color;
layout(location = 0) out vec3 v_normal;
layout(location = 1) out vec3 v_pos;
layout(location = 2) out vec4 v_color;
void main() {
    vec4 world_pos = model * vec4(position, 1.0);
    gl_Position = projection * view * world_pos;
    v_normal = mat3(model) * normal;
    v_pos = world_pos.xyz;
    v_color = color;
}
