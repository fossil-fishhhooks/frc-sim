#version 330
uniform vec4 model_color;
uniform vec4 ambient;
uniform vec4 light_pos;
uniform vec4 view_pos;
uniform float light_power;
uniform mat4 light_vp;
uniform sampler2D shadow_map;
in vec3 v_normal;
in vec3 v_pos;
in vec4 v_color;
out vec4 frag_color;

float shadow_factor(vec4 light_space) {
    vec3 proj = light_space.xyz / light_space.w;
    vec2 uv = proj.xy * 0.5 + 0.5;
    float current = proj.z * 0.5 + 0.5;
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0)
        return 1.0;
    float bias = 0.003;
    vec2 texel = 1.0 / textureSize(shadow_map, 0);
    float sum = 0.0;
    for (int x = -1; x <= 1; x++) {
        for (int y = -1; y <= 1; y++) {
            vec2 off = vec2(float(x), float(y)) * texel;
            float d = texture(shadow_map, uv + off).r;
            sum += (current - bias > d) ? 0.0 : 1.0;
        }
    }
    return sum / 9.0;
}

void main() {
    vec3 N = normalize(v_normal);
    vec3 Lv = light_pos.xyz - v_pos;
    float dist = length(Lv);
    vec3 L = Lv / dist;
    float atten = 1.0 / (1.0 + 0.007 * dist * dist);
    float diff = max(dot(N, L), 0.0);
    float shadow = 1.0;
    //if (diff > 0.0) shadow = shadow_factor(light_vp * vec4(v_pos, 1.0));
    vec3 V = normalize(view_pos.xyz - v_pos);
    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), 32.0);
    vec3 base = v_color.xyz * model_color.xyz;
    vec3 amb = ambient.xyz * base;
    vec3 dif = diff * base * light_power * atten * shadow;
    vec3 spe = spec * vec3(light_power * 0.2) * shadow;
    frag_color = vec4(amb + dif + spe, v_color.a * model_color.a);
}
