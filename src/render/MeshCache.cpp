#include "render/MeshCache.h"
#include "io/EasyLog.h"

#include "external/cgltf.h"
#include <vector>
#include <cmath>
#include <cstring>

struct Vertex {
    float x, y, z;
    float nx, ny, nz;
    uint8_t r, g, b, a;
};

static void mat4_mul_vec3(const float m[16], const float v[3], float out[3]) {
    out[0] = m[0]*v[0] + m[4]*v[1] + m[8]*v[2] + m[12];
    out[1] = m[1]*v[0] + m[5]*v[1] + m[9]*v[2] + m[13];
    out[2] = m[2]*v[0] + m[6]*v[1] + m[10]*v[2] + m[14];
}

static void mat4_mul_vec3_dir(const float m[16], const float v[3], float out[3]) {
    out[0] = m[0]*v[0] + m[4]*v[1] + m[8]*v[2];
    out[1] = m[1]*v[0] + m[5]*v[1] + m[9]*v[2];
    out[2] = m[2]*v[0] + m[6]*v[1] + m[10]*v[2];
}

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

// ── Public API ──────────────────────────────────────────────────────────────────

void MeshCache::Preload(const BodyDef* def) {
    if (!def) return;
    std::string key = def->mesh_path;
    if (key.empty()) return;
    if (m_cache.count(key)) return;
    CachedMesh mesh;
    if (LoadGLB(key, mesh))
        m_cache[key] = std::move(mesh);
    else
        LOG_ERROR("MeshCache: failed to load mesh %s", key.c_str());
}

const CachedMesh* MeshCache::Get(const BodyDef* def) const {
    if (!def) return nullptr;
    auto it = m_cache.find(def->mesh_path);
    return it != m_cache.end() ? &it->second : nullptr;
}

void MeshCache::UnloadAll() {
    for (auto& [key, mesh] : m_cache) {
        if (mesh.vertex_buf.id) sg_destroy_buffer(mesh.vertex_buf);
        if (mesh.index_buf.id) sg_destroy_buffer(mesh.index_buf);
    }
    m_cache.clear();
}

// ── GLB loading ─────────────────────────────────────────────────────────────────

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
    out.has_vertex_colors = false;

    struct ProcessNode {
        cgltf_data* data;
        std::vector<Vertex>& verts;
        std::vector<uint32_t>& indices;
        bool& has_vc;
        CachedMesh* mesh_out;
        size_t base = 0;

        void apply(cgltf_node* node) {
            float world[16];
            cgltf_node_transform_world(node, world);

            // Negative determinant on the upper-left 3x3 means this node's
            // transform mirrors space (e.g. a mirrored left/right robot
            // part via negative scale). Mirroring flips effective triangle
            // winding, so normals computed from (still CCW-authored) local
            // index order need to be flipped to stay outward-facing.
            float det3 =
                world[0] * (world[5]*world[10] - world[6]*world[9]) -
                world[4] * (world[1]*world[10] - world[2]*world[9]) +
                world[8] * (world[1]*world[6]  - world[2]*world[5]);
            float normal_sign = (det3 < 0.0f) ? -1.0f : 1.0f;

            if (node->mesh) {
                for (size_t p = 0; p < node->mesh->primitives_count; ++p) {
                    auto& prim = node->mesh->primitives[p];

                    // Compute material color for this primitive
                    float prim_color[4] = {1, 1, 1, 1};
                    if (prim.material && prim.material->has_pbr_metallic_roughness) {
                        auto* pbr = &prim.material->pbr_metallic_roughness;
                        prim_color[0] = pbr->base_color_factor[0];
                        prim_color[1] = pbr->base_color_factor[1];
                        prim_color[2] = pbr->base_color_factor[2];
                        prim_color[3] = pbr->base_color_factor[3];
                    }

                    const cgltf_accessor* pos_acc = nullptr;
                    const cgltf_accessor* norm_acc = nullptr;
                    const cgltf_accessor* col_acc = nullptr;
                    for (size_t a = 0; a < prim.attributes_count; ++a) {
                        auto& attr = prim.attributes[a];
                        if (attr.type == cgltf_attribute_type_position) pos_acc = attr.data;
                        if (attr.type == cgltf_attribute_type_normal) norm_acc = attr.data;
                        if (attr.type == cgltf_attribute_type_color) col_acc = attr.data;
                    }
                    if (!pos_acc) continue;

                    size_t v_off = verts.size();
                    verts.resize(v_off + pos_acc->count);
                    for (size_t i = 0; i < pos_acc->count; ++i) {
                        float v[3];
                        cgltf_accessor_read_float(pos_acc, i, v, 3);
                        float tv[3];
                        mat4_mul_vec3(world, v, tv);
                        verts[v_off + i] = {tv[0], tv[1], tv[2], 0, 0, 0, 255, 255, 255, 255};
                    }

                    if (col_acc) {
                        has_vc = true;
                        if (col_acc->type == cgltf_type_vec4) {
                            for (size_t i = 0; i < col_acc->count && i < pos_acc->count; ++i) {
                                float c[4];
                                cgltf_accessor_read_float(col_acc, i, c, 4);
                                verts[v_off + i].r = (uint8_t)(c[0] * 255.0f);
                                verts[v_off + i].g = (uint8_t)(c[1] * 255.0f);
                                verts[v_off + i].b = (uint8_t)(c[2] * 255.0f);
                                verts[v_off + i].a = (uint8_t)(c[3] * 255.0f);
                            }
                        } else if (col_acc->type == cgltf_type_vec3) {
                            for (size_t i = 0; i < col_acc->count && i < pos_acc->count; ++i) {
                                float c[3];
                                cgltf_accessor_read_float(col_acc, i, c, 3);
                                verts[v_off + i].r = (uint8_t)(c[0] * 255.0f);
                                verts[v_off + i].g = (uint8_t)(c[1] * 255.0f);
                                verts[v_off + i].b = (uint8_t)(c[2] * 255.0f);
                                verts[v_off + i].a = 255;
                            }
                        }
                    }

                    if (norm_acc) {
                        for (size_t i = 0; i < norm_acc->count && i < pos_acc->count; ++i) {
                            float n[3];
                            cgltf_accessor_read_float(norm_acc, i, n, 3);
                            float tn[3];
                            mat4_mul_vec3_dir(world, n, tn);
                            float len = sqrtf(tn[0]*tn[0] + tn[1]*tn[1] + tn[2]*tn[2]);
                            if (len > 1e-8f) { tn[0]/=len; tn[1]/=len; tn[2]/=len; }
                            verts[v_off + i].nx = tn[0];
                            verts[v_off + i].ny = tn[1];
                            verts[v_off + i].nz = tn[2];
                        }
                    }

                    size_t prim_base = base + v_off;
                    size_t idx_off = indices.size();
                    if (prim.indices) {
                        size_t icnt = prim.indices->count;
                        indices.resize(idx_off + icnt);
                        for (size_t i = 0; i < icnt; ++i)
                            indices[idx_off + i] = (uint32_t)(prim_base + cgltf_accessor_read_index(prim.indices, i));
                    } else {
                        for (size_t i = 0; i < pos_acc->count; i += 3) {
                            if (i+2 < pos_acc->count) {
                                indices.push_back((uint32_t)(prim_base + i));
                                indices.push_back((uint32_t)(prim_base + i+1));
                                indices.push_back((uint32_t)(prim_base + i+2));
                            }
                        }
                    }

                    // Record primitive range with material color
                    PrimitiveRange range = {};
                    range.index_offset = (int)idx_off;
                    range.index_count = (int)(indices.size() - idx_off);
                    memcpy(range.color, prim_color, sizeof(float[4]));
                    mesh_out->ranges.push_back(range);

                    if (!norm_acc) {
                        for (size_t i = idx_off; i < indices.size(); i += 3) {
                            uint32_t ia = indices[i], ib = indices[i+1], ic = indices[i+2];
                            float nrm[3];
                            if (compute_normal(&verts[ia].x, &verts[ib].x, &verts[ic].x, nrm)) {
                                verts[ia].nx += nrm[0]*normal_sign; verts[ia].ny += nrm[1]*normal_sign; verts[ia].nz += nrm[2]*normal_sign;
                                verts[ib].nx += nrm[0]*normal_sign; verts[ib].ny += nrm[1]*normal_sign; verts[ib].nz += nrm[2]*normal_sign;
                                verts[ic].nx += nrm[0]*normal_sign; verts[ic].ny += nrm[1]*normal_sign; verts[ic].nz += nrm[2]*normal_sign;
                            }
                        }
                        for (size_t i = 0; i < pos_acc->count; ++i) {
                            Vertex& vt = verts[v_off + i];
                            float len = sqrtf(vt.nx*vt.nx + vt.ny*vt.ny + vt.nz*vt.nz);
                            if (len > 1e-8f) { vt.nx /= len; vt.ny /= len; vt.nz /= len; }
                        }
                    }
                }
            }

            for (int i = 0; i < node->children_count; ++i)
                apply(node->children[i]);
        }
    };

    ProcessNode pn = {data, verts, indices, out.has_vertex_colors, &out};

    if (data->scene) {
        for (int i = 0; i < data->scene->nodes_count; ++i)
            pn.apply(data->scene->nodes[i]);
    } else {
        // No scene graph — iterate meshes directly
        for (size_t m = 0; m < data->meshes_count; ++m) {
            for (size_t p = 0; p < data->meshes[m].primitives_count; ++p) {
                auto& prim = data->meshes[m].primitives[p];

                float prim_color[4] = {1, 1, 1, 1};
                if (prim.material && prim.material->has_pbr_metallic_roughness) {
                    auto* pbr = &prim.material->pbr_metallic_roughness;
                    prim_color[0] = pbr->base_color_factor[0];
                    prim_color[1] = pbr->base_color_factor[1];
                    prim_color[2] = pbr->base_color_factor[2];
                    prim_color[3] = pbr->base_color_factor[3];
                }

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
                    verts[vcnt + i] = {v[0], v[1], v[2], 0, 0, 0, 255, 255, 255, 255};
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

                size_t idx_off_fb = indices.size();
                if (prim.indices) {
                    size_t icnt = prim.indices->count;
                    indices.resize(idx_off_fb + icnt);
                    for (size_t i = 0; i < icnt; ++i)
                        indices[idx_off_fb + i] = (uint32_t)(vcnt + cgltf_accessor_read_index(prim.indices, i));
                } else {
                    for (size_t i = 0; i < pos->count; i += 3) {
                        if (i+2 < pos->count) {
                            indices.push_back((uint32_t)(vcnt + i));
                            indices.push_back((uint32_t)(vcnt + i+1));
                            indices.push_back((uint32_t)(vcnt + i+2));
                        }
                    }
                }

                // Record primitive range with material color
                PrimitiveRange range = {};
                range.index_offset = (int)idx_off_fb;
                range.index_count = (int)(indices.size() - idx_off_fb);
                memcpy(range.color, prim_color, sizeof(float[4]));
                out.ranges.push_back(range);

                if (!norm) {
                    for (size_t i = idx_off_fb; i < indices.size(); i += 3) {
                        uint32_t ia = indices[i], ib = indices[i+1], ic = indices[i+2];
                        float nrm[3];
                        if (compute_normal(&verts[ia].x, &verts[ib].x, &verts[ic].x, nrm)) {
                            verts[ia].nx += nrm[0]; verts[ia].ny += nrm[1]; verts[ia].nz += nrm[2];
                            verts[ib].nx += nrm[0]; verts[ib].ny += nrm[1]; verts[ib].nz += nrm[2];
                            verts[ic].nx += nrm[0]; verts[ic].ny += nrm[1]; verts[ic].nz += nrm[2];
                        }
                    }
                    for (size_t i = 0; i < pos->count; ++i) {
                        Vertex& vt = verts[vcnt + i];
                        float len = sqrtf(vt.nx*vt.nx + vt.ny*vt.ny + vt.nz*vt.nz);
                        if (len > 1e-8f) { vt.nx /= len; vt.ny /= len; vt.nz /= len; }
                    }
                }
            }
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