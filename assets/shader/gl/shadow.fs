#version 330
out vec4 frag_color;
void main() {
    float d = gl_FragCoord.z;
    frag_color = vec4(d, d, d, 1.0);
}
