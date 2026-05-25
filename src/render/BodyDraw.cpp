#include "render/BodyDraw.h"
#include "io/EasyLog.h"

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <unordered_map>

static std::unordered_map<const BodyDef *, Model> s_cache;

void PreloadMesh(const BodyDef *def)
{
    if (!def || def->mesh_path.empty())
        return;
    if (s_cache.count(def))
        return;

    Model m = LoadModel(def->mesh_path.c_str());
    for (int i = 0; i < m.materialCount; ++i)
        LOG_INFO("BodyDraw: debug texture info: mat[%d] tex_id=%d w=%d h=%d color={%d,%d,%d}",
                 i,
                 m.materials[i].maps[MATERIAL_MAP_DIFFUSE].texture.id,
                 m.materials[i].maps[MATERIAL_MAP_DIFFUSE].texture.width,
                 m.materials[i].maps[MATERIAL_MAP_DIFFUSE].texture.height,
                 m.materials[i].maps[MATERIAL_MAP_DIFFUSE].color.r,
                 m.materials[i].maps[MATERIAL_MAP_DIFFUSE].color.g,
                 m.materials[i].maps[MATERIAL_MAP_DIFFUSE].color.b);
    if (m.meshCount == 0)
    {
        LOG_WARN("BodyDraw: failed to load model: %s", def->mesh_path.c_str());
        UnloadModel(m);
        return;
    }
    s_cache[def] = m;
    LOG_DEBUG("BodyDraw: cached model for '%s'", def->name.c_str());
}

void UnloadAllMeshes()
{
    for (auto &[key, model] : s_cache)
        UnloadModel(model);
    s_cache.clear();
}

/// reuse code from Shopping Poroject
static Matrix QuatToMatrix(const float q[4])
{
    float x = q[0], y = q[1], z = q[2], w = q[3];
    return Matrix{
        1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), 0,
        2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x), 0,
        2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y), 0,
        0, 0, 0, 1};
}

static Color BodyColor(const BodyDef *def)
{
    if (!def)
        return GRAY;
    // Static bodies: muted gray
    if (def->mass == 0.0f)
        return {255, 255, 255, 255};
    // Dynamic: stable color from name hash
    unsigned h = 2166136261u;
    for (char c : def->name)
        h = (h ^ (unsigned char)c) * 16777619u;
    return {
        (unsigned char)(120 + (h & 0x7F)),
        (unsigned char)(120 + ((h >> 8) & 0x7F)),
        (unsigned char)(120 + ((h >> 16) & 0x7F)),
        255};
}

void DrawBodySnapshot(const BodySnapshot &body, Shader *shader, bool wireframe)
{
    Vector3 pos = {body.pos[0], body.pos[1], body.pos[2]};
    Matrix rot = QuatToMatrix(body.rot);
    Color col = BodyColor(body.def);
    Matrix transform = MatrixMultiply(rot, MatrixTranslate(pos.x, pos.y, pos.z));

    auto it = body.def ? s_cache.find(body.def) : s_cache.end();
    if (it == s_cache.end() && body.def && !body.def->mesh_path.empty())
    {
        PreloadMesh(body.def);
        it = s_cache.find(body.def);
    }

    if (it != s_cache.end())
    {
        Model &model = it->second;
        for (int m = 0; m < model.meshCount; ++m)
        {
            Material &mat = model.materials[model.meshMaterial[m]];
            if (shader)
                mat.shader = *shader;

            // Only apply hash color if the mesh has no embedded texture.
            // GLBs with textures have a valid texture ID (> 0) in the diffuse slot —
            // overwriting colDiffuse with a hash color would tint the texture gray.
            bool has_texture = mat.maps[MATERIAL_MAP_DIFFUSE].texture.id > 0 && mat.maps[MATERIAL_MAP_DIFFUSE].texture.id != rlGetTextureIdDefault();
            if (!has_texture)
                mat.maps[MATERIAL_MAP_DIFFUSE].color = col;
            // if has_texture: leave color alone — whatever the GLB embedded is used

            DrawMesh(model.meshes[m], mat, transform);
        }

        // Wireframe pass — always default shader, no lighting
        if(wireframe){
            static Material wire_mat = LoadMaterialDefault();
            wire_mat.maps[MATERIAL_MAP_DIFFUSE].color = {30, 30, 30, 130};
            rlPushMatrix();
            rlMultMatrixf(MatrixToFloat(transform));
            rlEnableWireMode();
            for (int m = 0; m < model.meshCount; ++m)
                DrawMesh(model.meshes[m], wire_mat, MatrixIdentity());
            rlDisableWireMode();
            rlPopMatrix();
        }
    }
    else
    {
        DrawCube(pos, 0.3f, 0.3f, 0.3f, col);
        DrawCubeWires(pos, 0.3f, 0.3f, 0.3f, {40, 40, 40, 180});
    }
}