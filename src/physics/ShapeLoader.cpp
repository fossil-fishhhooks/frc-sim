#include "physics/ShapeLoader.h"
#include "io/EasyLog.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>

#include <cgltf.h>

#include <vector>

static std::vector<JPH::Vec3> ExtractVertices(cgltf_data* data)
{
    std::vector<JPH::Vec3> verts;
    for (size_t m = 0; m < data->meshes_count; ++m) {
        const cgltf_mesh& mesh = data->meshes[m];
        for (size_t p = 0; p < mesh.primitives_count; ++p) {
            const cgltf_primitive& prim = mesh.primitives[p];

            const cgltf_accessor* pos = nullptr;
            for (size_t a = 0; a < prim.attributes_count; ++a) {
                if (prim.attributes[a].type == cgltf_attribute_type_position) {
                    pos = prim.attributes[a].data;
                    break;
                }
            }
            if (!pos) continue;

            size_t cnt = verts.size();
            verts.reserve(cnt + pos->count);
            for (size_t i = 0; i < pos->count; ++i) {
                float v[3];
                cgltf_accessor_read_float(pos, i, v, 3);
                verts.emplace_back(v[0], v[1], v[2]);
            }
        }
    }
    return verts;
}

static JPH::TriangleList ExtractTriangles(cgltf_data* data,
                                          const std::vector<JPH::Vec3>& verts)
{
    JPH::TriangleList tris;
    size_t base = 0;

    for (size_t m = 0; m < data->meshes_count; ++m) {
        const cgltf_mesh& mesh = data->meshes[m];
        for (size_t p = 0; p < mesh.primitives_count; ++p) {
            const cgltf_primitive& prim = mesh.primitives[p];

            const cgltf_accessor* pos = nullptr;
            for (size_t a = 0; a < prim.attributes_count; ++a) {
                if (prim.attributes[a].type == cgltf_attribute_type_position) {
                    pos = prim.attributes[a].data;
                    break;
                }
            }
            if (!pos) continue;

            auto make_tri = [&](size_t a, size_t b, size_t c) {
                size_t ga = base + a, gb = base + b, gc = base + c;
                if (ga >= verts.size() || gb >= verts.size() || gc >= verts.size())
                    return;

                tris.push_back(JPH::Triangle(
                    JPH::Float3(verts[ga].GetX(), verts[ga].GetY(), verts[ga].GetZ()),
                    JPH::Float3(verts[gb].GetX(), verts[gb].GetY(), verts[gb].GetZ()),
                    JPH::Float3(verts[gc].GetX(), verts[gc].GetY(), verts[gc].GetZ())
                ));
            };

            if (prim.indices) {
                size_t tri_count = prim.indices->count / 3;
                for (size_t i = 0; i < tri_count; ++i) {
                    make_tri(
                        (size_t)cgltf_accessor_read_index(prim.indices, i * 3 + 0),
                        (size_t)cgltf_accessor_read_index(prim.indices, i * 3 + 1),
                        (size_t)cgltf_accessor_read_index(prim.indices, i * 3 + 2)
                    );
                }
            } else {
                for (size_t i = 0; i < pos->count; i += 3)
                    make_tri(i, i + 1, i + 2);
            }

            base += pos->count;
        }
    }

    return tris;
}


JPH::Ref<JPH::Shape> LoadShape(const std::string& mesh_path, bool is_static)
{
    cgltf_options options = {};
    cgltf_data* data = nullptr;

    cgltf_result cerr = cgltf_parse_file(&options, mesh_path.c_str(), &data);
    if (cerr != cgltf_result_success) {
        LOG_ERROR("ShapeLoader: failed to parse mesh: %s", mesh_path.c_str());
        return nullptr;
    }

    cerr = cgltf_load_buffers(&options, data, mesh_path.c_str());
    if (cerr != cgltf_result_success) {
        LOG_ERROR("ShapeLoader: failed to load buffers for: %s", mesh_path.c_str());
        cgltf_free(data);
        return nullptr;
    }

    if (data->meshes_count == 0) {
        LOG_ERROR("ShapeLoader: no meshes in: %s", mesh_path.c_str());
        cgltf_free(data);
        return nullptr;
    }

    auto verts = ExtractVertices(data);
    if (verts.empty()) {
        LOG_ERROR("ShapeLoader: no vertices in: %s", mesh_path.c_str());
        cgltf_free(data);
        return nullptr;
    }

    JPH::Ref<JPH::Shape> shape;

    if (is_static) {

        auto tris = ExtractTriangles(data, verts);
        if (tris.empty()) {
            LOG_ERROR("ShapeLoader: no triangles extracted from: %s", mesh_path.c_str());
            cgltf_free(data);
            return nullptr;
        }

        JPH::MeshShapeSettings settings(std::move(tris));
        auto result = settings.Create();
        if (result.HasError()) {
            LOG_ERROR("ShapeLoader: MeshShape error for %s: %s",
                      mesh_path.c_str(), result.GetError().c_str());
        } else {
            shape = result.Get();
            LOG_DEBUG("ShapeLoader: MeshShape built from %s (%zu verts)",
                      mesh_path.c_str(), verts.size());
        }

    } else {

        JPH::Array<JPH::Vec3> jph_verts(verts.data(), verts.data() + verts.size());
        JPH::ConvexHullShapeSettings settings(jph_verts);
        settings.mMaxConvexRadius = 0.01f;

        auto result = settings.Create();
        if (result.HasError()) {
            LOG_ERROR("ShapeLoader: ConvexHullShape error for %s: %s",
                      mesh_path.c_str(), result.GetError().c_str());
        } else {
            shape = result.Get();
            LOG_DEBUG("ShapeLoader: ConvexHullShape built from %s (%zu verts)",
                      mesh_path.c_str(), verts.size());
        }
    }

    cgltf_free(data);
    return shape;
}
