#version 450
layout(std140, binding = 0) uniform shadow_vs_params {
    mat4 light_vp;
    mat4 model;
};
layout(location = 0) in vec3 position;
void main() {
    gl_Position = light_vp * model * vec4(position, 1.0);
}
