#pragma once
#include "core/BodyDef.h"
#include "core/Snapshot.h"
#include <sokol_gfx.h>
#include <string>
#include <unordered_map>
#include <vector>

struct PrimitiveRange {
    int index_offset;
    int index_count;
    float color[4];
};

struct CachedMesh {
    sg_buffer vertex_buf = {};
    sg_buffer index_buf = {};
    int num_indices = 0;
    bool valid = false;
    bool has_vertex_colors = false;
    std::vector<PrimitiveRange> ranges;
};

class MeshCache {
public:
    void Preload(const BodyDef* def);
    void UnloadAll();
    const CachedMesh* Get(const BodyDef* def) const;

private:
    std::unordered_map<std::string, CachedMesh> m_cache;
    bool LoadGLB(const std::string& path, CachedMesh& out);
};
