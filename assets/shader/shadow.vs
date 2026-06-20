#version 330
uniform mat4 light_vp;
uniform mat4 model;
layout(location = 0) in vec3 position;
void main() {
    gl_Position = light_vp * model * vec4(position, 1.0);
}
