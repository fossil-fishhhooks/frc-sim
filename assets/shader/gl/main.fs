#version 330
uniform vec4 model_color;
uniform vec4 ambient;
uniform vec4 light_pos;
uniform vec4 view_pos;
uniform mat4 light_vp;
uniform float light_power;
uniform sampler2DShadow shadow_map;
in vec3 v_normal;
in vec3 v_pos;
in vec4 v_color;
out vec4 frag_color;

float ShadowPCF(vec3 world_pos) {
    vec4 lp   = light_vp * vec4(world_pos, 1.0);
    vec3 proj = lp.xyz / lp.w * 0.5 + 0.5;
    if (proj.z >= 1.0) return 1.0;
    vec2 texel = 1.0 / vec2(4096.0);
    float shadow = 0.0;
    for (int x = -2; x <= 2; ++x)
        for (int y = -2; y <= 2; ++y)
            shadow += texture(shadow_map, vec3(proj.xy + vec2(x, y) * texel, proj.z - 0.005));
    return shadow / 25.0;
}

void main() {
    vec3 N    = normalize(v_normal);
    vec3 Lv   = light_pos.xyz - v_pos;
    float dist = length(Lv);
    vec3 L    = Lv / dist;
    float atten = 1.0 / (1.0 + 0.007 * dist * dist);
    float diff  = max(dot(N, L), 0.0);
    vec3 base   = model_color.rgb * v_color.rgb;
    vec3 amb    = ambient.rgb * base;
    float shadow = ShadowPCF(v_pos);
    vec3 diffuse = diff * atten * base * light_power * shadow;
    vec3 V    = normalize(view_pos.xyz - v_pos);
    vec3 H    = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), 32.0) * atten * light_power * shadow;
    frag_color = vec4(amb + diffuse + vec3(spec * 0.3), 1.0);
}
