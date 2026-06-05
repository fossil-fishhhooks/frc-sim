#include "physics/ShapeLoader.h"
#include "io/EasyLog.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>

// Raylib included AFTER Jolt to avoid macro conflicts
#include <raylib.h>

#include <vector>

//Mesh extraction

static std::vector<JPH::Vec3> ExtractVertices(const Model& model)
{
    std::vector<JPH::Vec3> verts;
    for (int m = 0; m < model.meshCount; ++m) {
        const Mesh& mesh = model.meshes[m];
        float* v = mesh.vertices;
        verts.reserve(verts.size() + mesh.vertexCount);
        for (int i = 0; i < mesh.vertexCount; ++i)
            verts.emplace_back(v[i*3+0], v[i*3+1], v[i*3+2]);
    }
    return verts;
}

// Returns flat triangle list. Vertex indices are global (offset by mesh base).
static JPH::TriangleList ExtractTriangles(const Model& model,
                                          const std::vector<JPH::Vec3>& verts)
{
    JPH::TriangleList tris;
    int base = 0;

    for (int m = 0; m < model.meshCount; ++m) {
        const Mesh& mesh = model.meshes[m];

        auto make_tri = [&](int a, int b, int c) {
            int ga = base + a, gb = base + b, gc = base + c;
            if (ga >= (int)verts.size() ||
                gb >= (int)verts.size() ||
                gc >= (int)verts.size()) return;

            tris.push_back(JPH::Triangle(
                JPH::Float3(verts[ga].GetX(), verts[ga].GetY(), verts[ga].GetZ()),
                                         JPH::Float3(verts[gb].GetX(), verts[gb].GetY(), verts[gb].GetZ()),
                                         JPH::Float3(verts[gc].GetX(), verts[gc].GetY(), verts[gc].GetZ())
            ));
        };

        if (mesh.indices) {
            for (int i = 0; i < mesh.triangleCount; ++i)
                make_tri(mesh.indices[i*3+0],
                         mesh.indices[i*3+1],
                         mesh.indices[i*3+2]);
        } else {
            for (int i = 0; i < mesh.vertexCount; i += 3)
                make_tri(i, i+1, i+2);
        }

        base += mesh.vertexCount;
    }
    return tris;
}


JPH::Ref<JPH::Shape> LoadShape(const std::string& mesh_path, bool is_static)
{
    Model model = LoadModel(mesh_path.c_str());
    if (model.meshCount == 0) {
        LOG_ERROR("ShapeLoader: failed to load mesh: %s", mesh_path.c_str());
        UnloadModel(model);
        return nullptr;
    }

    auto verts = ExtractVertices(model);
    if (verts.empty()) {
        LOG_ERROR("ShapeLoader: no vertices in mesh: %s", mesh_path.c_str());
        UnloadModel(model);
        return nullptr;
    }
    

    JPH::Ref<JPH::Shape> shape;

    if (is_static) {

        auto tris = ExtractTriangles(model, verts);
        if (tris.empty()) {
            LOG_ERROR("ShapeLoader: no triangles extracted from: %s", mesh_path.c_str());
            UnloadModel(model);
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
        settings.mMaxConvexRadius = 0.01f;   // 1 cm

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

    UnloadModel(model);
    return shape;
}
