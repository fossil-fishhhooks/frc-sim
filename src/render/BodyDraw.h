#pragma once
#include "core/Snapshot.h"
#include "core/BodyDef.h"
#include "render/MeshCache.h"

#include <sokol_gfx.h>

void PreloadMesh(const BodyDef* def, MeshCache* cache);
void UnloadAllMeshes();

void DrawBodyRange(const BodySnapshot& body, MeshCache* cache,
                   int idx_offset, int idx_count);

// shared matrix helpers
void QuatToMatrix(const float q[4], float out[16]);
static inline void Vec3Add(const float a[3], const float b[3], float out[3]) {
    out[0]=a[0]+b[0]; out[1]=a[1]+b[1]; out[2]=a[2]+b[2]; }
static inline void Vec3Scale(const float a[3], float s, float out[3]) {
    out[0]=a[0]*s; out[1]=a[1]*s; out[2]=a[2]*s; }
void Vec3RotateByQuat(const float v[3], const float q[4], float out[3]);
void BodyColor(const BodyDef* def, float out[4]);
