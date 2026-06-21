#version 450
layout(std140, binding = 1) uniform fs_params {
    vec4 model_color;
    vec4 ambient;
    vec4 light_pos;
    vec4 view_pos;
    mat4 light_vp;
    float light_power;
};
layout(set = 1, binding = 0) uniform sampler2D shadow_map;
layout(location = 0) in vec3 v_normal;
layout(location = 1) in vec3 v_pos;
layout(location = 2) in vec4 v_color;
layout(location = 0) out vec4 frag_color;
void main() {
    vec3 N = normalize(v_normal);
    vec3 Lv = light_pos.xyz - v_pos;
    float dist = length(Lv);
    vec3 L = Lv / dist;
    float atten = 1.0 / (1.0 + 0.007 * dist * dist);
    float diff = max(dot(N, L), 0.0);
    vec3 base = model_color.rgb * v_color.rgb;
    vec3 amb = ambient.rgb * base;
    vec3 diffuse = diff * atten * base * light_power;
    vec3 V = normalize(view_pos.xyz - v_pos);
    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), 32.0) * atten * light_power;
    frag_color = vec4(amb + diffuse + vec3(spec * 0.3), 1.0);
}
