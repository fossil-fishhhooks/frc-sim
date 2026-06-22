#include <metal_stdlib>
using namespace metal;

struct ShadowVsParams {
    float4x4 light_vp;
    float4x4 model;
};

struct VertexIn {
    float3 position [[attribute(0)]];
};

vertex float4 shadow_vs(VertexIn in [[stage_in]],
                         constant ShadowVsParams& params [[buffer(0)]]) {
    return params.light_vp * params.model * float4(in.position, 1.0);
}

// Depth-only pass — no color output.
fragment void shadow_fs(float4 frag_coord [[position]]) {}
