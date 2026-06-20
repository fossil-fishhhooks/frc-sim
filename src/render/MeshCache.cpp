#include "render/MeshCache.h"
#include "io/EasyLog.h"

#include "external/cgltf.h"
#include <vector>
#include <cmath>

void MeshCache::Preload(const BodyDef* def) {
    if (!def || def->mesh_path.empty()) return;
    if (m_cache.count(def)) return;

    CachedMesh cm;
    if (LoadGLB(def->mesh_path, cm)) {
        m_cache[def] = cm;
        LOG_DEBUG("MeshCache: cached '%s' (%d indices)", def->name.c_str(), cm.num_indices);
    }
}

const CachedMesh* MeshCache::Get(const BodyDef* def) const {
    auto it = m_cache.find(def);
    return it != m_cache.end() ? &it->second : nullptr;
}

void MeshCache::UnloadAll() {
    for (auto& [key, cm] : m_cache) {
        if (cm.vertex_buf.id) sg_destroy_buffer(cm.vertex_buf);
        if (cm.index_buf.id) sg_destroy_buffer(cm.index_buf);
    }
    m_cache.clear();
}

struct Vertex {
    float x, y, z;
    float nx, ny, nz;
};

static bool compute_normal(const float a[3], const float b[3], const float c[3],
                           float out[3]) {
    float u[3] = {b[0]-a[0], b[1]-a[1], b[2]-a[2]};
    float v[3] = {c[0]-a[0], c[1]-a[1], c[2]-a[2]};
    out[0] = u[1]*v[2] - u[2]*v[1];
    out[1] = u[2]*v[0] - u[0]*v[2];
    out[2] = u[0]*v[1] - u[1]*v[0];
    float len = sqrtf(out[0]*out[0] + out[1]*out[1] + out[2]*out[2]);
    if (len < 1e-8f) return false;
    out[0]/=len; out[1]/=len; out[2]/=len;
    return true;
}

bool MeshCache::LoadGLB(const std::string& path, CachedMesh& out) {
    cgltf_options opts = {};
    cgltf_data* data = nullptr;
    if (cgltf_parse_file(&opts, path.c_str(), &data) != cgltf_result_success) {
        LOG_ERROR("MeshCache: failed to parse %s", path.c_str());
        return false;
    }
    if (cgltf_load_buffers(&opts, data, path.c_str()) != cgltf_result_success) {
        LOG_ERROR("MeshCache: failed to load buffers %s", path.c_str());
        cgltf_free(data);
        return false;
    }

    std::vector<Vertex> verts;
    std::vector<uint32_t> indices;
    size_t base = 0;

    for (size_t m = 0; m < data->meshes_count; ++m) {
        const auto& mesh = data->meshes[m];
        for (size_t p = 0; p < mesh.primitives_count; ++p) {
            const auto& prim = mesh.primitives[p];

            const cgltf_accessor* pos = nullptr;
            const cgltf_accessor* norm = nullptr;
            for (size_t a = 0; a < prim.attributes_count; ++a) {
                auto& attr = prim.attributes[a];
                if (attr.type == cgltf_attribute_type_position) pos = attr.data;
                if (attr.type == cgltf_attribute_type_normal) norm = attr.data;
            }
            if (!pos) continue;

            size_t vcnt = verts.size();
            verts.resize(vcnt + pos->count);
            for (size_t i = 0; i < pos->count; ++i) {
                float v[3];
                cgltf_accessor_read_float(pos, i, v, 3);
                verts[vcnt + i] = {v[0], v[1], v[2], 0, 0, 0};
            }

            if (norm) {
                for (size_t i = 0; i < norm->count && i < pos->count; ++i) {
                    float n[3];
                    cgltf_accessor_read_float(norm, i, n, 3);
                    verts[vcnt + i].nx = n[0];
                    verts[vcnt + i].ny = n[1];
                    verts[vcnt + i].nz = n[2];
                }
            }

            if (prim.indices) {
                size_t icnt = prim.indices->count;
                size_t idx_off = indices.size();
                indices.resize(idx_off + icnt);
                for (size_t i = 0; i < icnt; ++i)
                    indices[idx_off + i] = (uint32_t)(base + cgltf_accessor_read_index(prim.indices, i));
            } else {
                for (size_t i = 0; i < pos->count; i += 3) {
                    if (i+2 < pos->count) {
                        indices.push_back((uint32_t)(base + i));
                        indices.push_back((uint32_t)(base + i+1));
                        indices.push_back((uint32_t)(base + i+2));
                    }
                }
            }

            if (!norm) {
                for (size_t i = 0; i + 2 < pos->count; i += 3) {
                    float nrm[3];
                    if (compute_normal(&verts[base+i].x, &verts[base+i+1].x, &verts[base+i+2].x, nrm)) {
                        verts[base+i].nx = nrm[0]; verts[base+i].ny = nrm[1]; verts[base+i].nz = nrm[2];
                        verts[base+i+1].nx = nrm[0]; verts[base+i+1].ny = nrm[1]; verts[base+i+1].nz = nrm[2];
                        verts[base+i+2].nx = nrm[0]; verts[base+i+2].ny = nrm[1]; verts[base+i+2].nz = nrm[2];
                    }
                }
            }

            base += pos->count;
        }
    }

    if (verts.empty() || indices.empty()) {
        LOG_ERROR("MeshCache: no geometry in %s", path.c_str());
        cgltf_free(data);
        return false;
    }

    sg_buffer_desc vdesc = {};
    vdesc.usage.vertex_buffer = true;
    vdesc.usage.immutable = true;
    vdesc.data.ptr = verts.data();
    vdesc.data.size = verts.size() * sizeof(Vertex);
    out.vertex_buf = sg_make_buffer(&vdesc);

    sg_buffer_desc idesc = {};
    idesc.usage.index_buffer = true;
    idesc.usage.immutable = true;
    idesc.data.ptr = indices.data();
    idesc.data.size = indices.size() * sizeof(uint32_t);
    out.index_buf = sg_make_buffer(&idesc);

    out.num_indices = (int)indices.size();
    out.valid = true;

    cgltf_free(data);
    return true;
}
