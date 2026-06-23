#include <metal_stdlib>
using namespace metal;

struct VsParams {
    float4x4 model;
    float4x4 view;
    float4x4 projection;
};

struct FsParams {
    float4   model_color;
    float4   ambient;
    float4   light_pos;
    float4   view_pos;
    float4x4 light_vp;
    float    light_power;
};

struct VertexIn {
    float3 position [[attribute(0)]];
    float3 normal   [[attribute(1)]];
    float4 color    [[attribute(2)]];
};

struct VertexOut {
    float4 position [[position]];
    float3 v_normal;
    float3 v_pos;
    float4 v_color;
};

vertex VertexOut main_vs(VertexIn in [[stage_in]],
                          constant VsParams& params [[buffer(0)]]) {
    VertexOut out;
    float4 world_pos = params.model * float4(in.position, 1.0);
    out.position = params.projection * params.view * world_pos;
    out.v_normal = (params.model * float4(in.normal, 0.0)).xyz;
    out.v_pos    = world_pos.xyz;
    out.v_color  = in.color;
    return out;
}

static float ShadowPCF(depth2d<float> shadow_map, sampler shadow_sampler,
                        float3 v_pos, float4x4 light_vp) {
    float4 lp   = light_vp * float4(v_pos, 1.0);
    float3 proj = lp.xyz / lp.w * 0.5 + 0.5;
    if (proj.z >= 1.0) return 1.0;
    float2 texel = 1.0 / float2(2048.0);
    float shadow = 0.0;
    for (int x = -2; x <= 2; ++x)
        for (int y = -2; y <= 2; ++y)
            shadow += shadow_map.sample_compare(shadow_sampler,
                          proj.xy + float2(x, y) * texel, proj.z - 0.005);
    return shadow / 25.0;
}

fragment float4 main_fs(VertexOut in [[stage_in]],
                         constant FsParams& params [[buffer(1)]],
                         depth2d<float> shadow_map     [[texture(0)]],
                         sampler        shadow_sampler  [[sampler(0)]]) {
    float3 N     = normalize(in.v_normal);
    float3 Lv    = params.light_pos.xyz - in.v_pos;
    float  dist  = length(Lv);
    float3 L     = Lv / dist;
    float atten  = 1.0 / (1.0 + 0.007 * dist * dist);
    float diff   = max(dot(N, L), 0.0);
    float3 base  = params.model_color.rgb * in.v_color.rgb;
    float3 amb   = params.ambient.rgb * base;
    float shadow = ShadowPCF(shadow_map, shadow_sampler, in.v_pos, params.light_vp);
    float3 diffuse = diff * atten * base * params.light_power * shadow;
    float3 V     = normalize(params.view_pos.xyz - in.v_pos);
    float3 H     = normalize(L + V);
    float spec   = pow(max(dot(N, H), 0.0), 32.0) * atten * params.light_power * shadow;
    return float4(amb + diffuse + float3(spec * 0.3), 1.0);
}
