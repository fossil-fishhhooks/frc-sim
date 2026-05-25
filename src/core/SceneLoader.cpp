#include "core/SceneLoader.h"
#include "core/BodyLoader.h"
#include "io/EasyLog.h"

#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <unordered_map>

using json = nlohmann::json;
namespace fs = std::filesystem;

static std::array<float, 3> readVec3(const json &j, const char *key,
                                     std::array<float, 3> def = {0, 0, 0})
{
    if (!j.contains(key))
        return def;
    const auto &v = j[key];
    return {v[0].get<float>(), v[1].get<float>(), v[2].get<float>()};
}

SceneData LoadScene(const std::string &scene_path, const MotorRegistry &motors)
{
    SceneData result;

    std::ifstream f(scene_path);
    if (!f.is_open())
    {
        LOG_ERROR("SceneLoader: cannot open: %s", scene_path.c_str());
        return result;
    }

    json j;
    try
    {
        f >> j;
    }
    catch (const json::parse_error &e)
    {
        LOG_ERROR("SceneLoader: parse error: %s", e.what());
        return result;
    }

    fs::path scene_dir = fs::path(scene_path).parent_path();
    std::string scene_name = j.value("name", "unnamed");
    LOG_INFO("SceneLoader: loading scene '%s'", scene_name.c_str());

    if (!j.contains("bodies"))
    {
        LOG_WARN("SceneLoader: scene '%s' has no 'bodies' array", scene_name.c_str());
        return result;
    }

    std::unordered_map<std::string, BodyDef> def_cache;

    for (const auto &entry : j["bodies"])
    {
        if (!entry.contains("def"))
        {
            LOG_WARN("SceneLoader: body entry missing 'def' field, skipping");
            continue;
        }

        std::string def_rel = entry["def"].get<std::string>();
        std::string def_path = (scene_dir / def_rel).string();

        if (!def_cache.count(def_path))
        {
            auto maybe = LoadBodyDef(def_path, motors);
            if (!maybe)
            {
                LOG_WARN("SceneLoader: skipping body with failed def: %s", def_path.c_str());
                continue;
            }
            def_cache[def_path] = std::move(*maybe);
        }

        SpawnRequest req;
        req.def = def_cache[def_path];
        req.role = entry.value("role", "");

        if (entry.contains("position"))
        {
            auto &p = entry["position"];
            req.position = {p[0].get<float>(), p[1].get<float>(), p[2].get<float>()};
        }
        if (entry.contains("orientation"))
        {
            auto &o = entry["orientation"];
            req.orientation = {o[0].get<float>(), o[1].get<float>(),
                               o[2].get<float>(), o[3].get<float>()};
        }

        result.bodies.push_back(std::move(req));
    }

    // ── Optional intake ───────────────────────────────────────────────────
    if (j.contains("intake"))
    {
        const auto &ji = j["intake"];
        result.has_intake = true;
        result.intake.center = readVec3(ji, "center");
        result.intake.half_extents = readVec3(ji, "half_extents",
                                              {0.2f, 0.2f, 0.2f});
        result.intake.max_capacity = ji.value("max_capacity", 3);
        if (ji.contains("orientation"))
        {
            auto &o = ji["orientation"];
            result.intake.orientation = {o[0].get<float>(), o[1].get<float>(),
                                         o[2].get<float>(), o[3].get<float>()};
        }
        LOG_INFO("SceneLoader: intake loaded  cap=%d", result.intake.max_capacity);
    }

    // ── Optional shooter ──────────────────────────────────────────────────
    if (j.contains("shooter"))
    {
        const auto &js = j["shooter"];
        result.shooter.exit_point = readVec3(js, "exit_point",
                                             {0.f, 0.5f, 0.3f});
        result.shooter.default_direction = readVec3(js, "default_direction",
                                                    {0.f, 0.707f, 0.707f});
        std::string piece_rel = js.value("piece_mesh_path", "");
        if (!piece_rel.empty())
        {
            std::string piece_def_path = (scene_dir / piece_rel).string();
            auto piece_def = LoadBodyDef(piece_def_path, motors);
            if (piece_def && !piece_def->mesh_path.empty())
            {
                result.shooter.piece_mesh_path = piece_def->mesh_path;
                result.shooter.piece_name = piece_def->name; // carry the name through
            }
            else
            {
                LOG_WARN("SceneLoader: shooter piece_mesh_path '%s' could not be resolved — shooter disabled",
                         piece_def_path.c_str());
            }
        }
        result.shooter.piece_mass = js.value("piece_mass", 0.25f);
        result.shooter.fire_rate = js.value("fire_rate", 2.0f);
        LOG_INFO("SceneLoader: shooter loaded  mesh='%s'",
                 result.shooter.piece_mesh_path.c_str());
    }

    LOG_INFO("SceneLoader: %zu spawn requests loaded from '%s'",
             result.bodies.size(), scene_name.c_str());
    return result;
}