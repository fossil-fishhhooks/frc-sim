#version 330
uniform vec4 model_color;
uniform vec4 ambient;
uniform vec4 light_pos;
uniform vec4 view_pos;
uniform float light_power;
in vec3 v_normal;
in vec3 v_pos;
in vec4 v_color;
out vec4 frag_color;
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
