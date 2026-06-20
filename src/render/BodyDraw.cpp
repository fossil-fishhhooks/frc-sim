#include "render/BodyDraw.h"
#include "io/EasyLog.h"

#include <cstring>
#include <cmath>

static unsigned FNV1a(const char* str) {
    unsigned h = 2166136261u;
    for (const char* p = str; *p; ++p)
        h = (h ^ (unsigned char)*p) * 16777619u;
    return h;
}

void BodyColor(const BodyDef* def, float out[4]) {
    if (!def) {
        out[0]=0.5f; out[1]=0.5f; out[2]=0.5f; out[3]=1.0f; return;
    }
    if (def->mass == 0.0f) {
        out[0]=1.0f; out[1]=1.0f; out[2]=1.0f; out[3]=1.0f; return;
    }
    unsigned h = FNV1a(def->name.c_str());
    out[0] = (120.0f + (h & 0x7F)) / 255.0f;
    out[1] = (120.0f + ((h >> 8) & 0x7F)) / 255.0f;
    out[2] = (120.0f + ((h >> 16) & 0x7F)) / 255.0f;
    out[3] = 1.0f;
}

void QuatToMatrix(const float q[4], float out[16]) {
    float x=q[0], y=q[1], z=q[2], w=q[3];
    memset(out, 0, 16*sizeof(float));
    out[0]  = 1 - 2*(y*y + z*z);
    out[1]  = 2*(x*y + w*z);
    out[2]  = 2*(x*z - w*y);
    out[4]  = 2*(x*y - w*z);
    out[5]  = 1 - 2*(x*x + z*z);
    out[6]  = 2*(y*z + w*x);
    out[8]  = 2*(x*z + w*y);
    out[9]  = 2*(y*z - w*x);
    out[10] = 1 - 2*(x*x + y*y);
    out[15] = 1;
}

void Vec3RotateByQuat(const float v[3], const float q[4], float out[3]) {
    float x=q[0], y=q[1], z=q[2], w=q[3];
    float tx = 2*(y*v[2] - z*v[1]);
    float ty = 2*(z*v[0] - x*v[2]);
    float tz = 2*(x*v[1] - y*v[0]);
    out[0] = v[0] + w*tx + (y*tz - z*ty);
    out[1] = v[1] + w*ty + (z*tx - x*tz);
    out[2] = v[2] + w*tz + (x*ty - y*tx);
}

void PreloadMesh(const BodyDef* def, MeshCache* cache) {
    if (cache) cache->Preload(def);
}

void UnloadAllMeshes() {
    // handled by MeshCache::UnloadAll
}

void DrawBodySnapshot(const BodySnapshot& body, MeshCache* cache, bool wireframe) {
    if (!cache) return;

    const CachedMesh* mesh = cache->Get(body.def);
    if (!mesh || !mesh->valid) {
        // No mesh loaded — draw nothing visible
        // In the future: draw a fallback placeholder
        return;
    }

    sg_bindings bind = {};
    bind.vertex_buffers[0] = mesh->vertex_buf;
    bind.index_buffer = mesh->index_buf;
    sg_apply_bindings(&bind);

    // Draw all triangles
    sg_draw(0, mesh->num_indices / 3, 1);
}
