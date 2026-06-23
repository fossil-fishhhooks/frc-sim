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

float SampleShadow(vec2 uv, float compare) {
    float d = texture(shadow_map, uv).r;
    return compare <= d ? 1.0 : 0.0;
}

float ShadowPCF(vec3 world_pos) {
    vec4 lp   = light_vp * vec4(world_pos, 1.0);
    vec3 proj = lp.xyz / lp.w;
    proj      = proj * 0.5 + 0.5;
    if (proj.z >= 1.0) return 1.0;
    vec2 texel = 1.0 / vec2(1024.0);
    float bias = 0.005;
    float shadow = 0.0;
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
            shadow += SampleShadow(proj.xy + vec2(x, y) * texel, proj.z - bias);
    return shadow / 9.0;
}

void main() {
    vec3 N     = normalize(v_normal);
    vec3 Lv    = light_pos.xyz - v_pos;
    float dist  = length(Lv);
    vec3 L     = Lv / dist;
    float atten = 1.0 / (1.0 + 0.007 * dist * dist);
    float diff  = max(dot(N, L), 0.0);
    vec3 base   = model_color.rgb * v_color.rgb;
    vec3 amb    = ambient.rgb * base;
    float shadow = ShadowPCF(v_pos);
    vec3 diffuse = diff * atten * base * light_power * shadow;
    vec3 V     = normalize(view_pos.xyz - v_pos);
    vec3 H     = normalize(L + V);
    float spec  = pow(max(dot(N, H), 0.0), 32.0) * atten * light_power * shadow;
    frag_color  = vec4(amb + diffuse + vec3(spec * 0.3), 1.0);
}
