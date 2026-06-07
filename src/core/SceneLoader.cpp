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
    if (!j.contains(key)) return def;
    const auto &v = j[key];
    return {v[0].get<float>(), v[1].get<float>(), v[2].get<float>()};
}

static std::array<float, 4> readVec4(const json &j, const char *key,
                                     std::array<float, 4> def = {0, 0, 0, 1})
{
    if (!j.contains(key)) return def;
    const auto &v = j[key];
    return {v[0].get<float>(), v[1].get<float>(),
            v[2].get<float>(), v[3].get<float>()};
}

// Parse intake + shooter defs from a JSON object into a RobotSpawn
static void ParseMechanisms(const json &j, const fs::path &scene_dir,
                             const MotorRegistry &motors, RobotSpawn &spawn)
{
    if (j.contains("intake"))
    {
        const auto &ji = j["intake"];
        spawn.has_mechanisms       = true;
        spawn.intake.center        = readVec3(ji, "center");
        spawn.intake.half_extents  = readVec3(ji, "half_extents", {0.2f, 0.2f, 0.2f});
        spawn.intake.max_capacity  = ji.value("max_capacity", 3);
        spawn.intake.orientation   = readVec4(ji, "orientation");
        LOG_INFO("SceneLoader: robot intake loaded  cap=%d", spawn.intake.max_capacity);
    }

    if (j.contains("shooter"))
    {
        const auto &js = j["shooter"];
        spawn.has_mechanisms            = true;
        spawn.shooter.exit_point        = readVec3(js, "exit_point",      {0.f, 0.5f, 0.3f});
        spawn.shooter.default_direction = readVec3(js, "default_direction", {0.f, 0.707f, 0.707f});
        spawn.shooter.piece_mass        = js.value("piece_mass", 0.25f);
        spawn.shooter.fire_rate         = js.value("fire_rate",  2.0f);

        std::string piece_rel = js.value("piece_mesh_path", "");
        if (!piece_rel.empty())
        {
            std::string piece_def_path = (scene_dir / piece_rel).string();
            auto piece_def = LoadBodyDef(piece_def_path, motors);
            if (piece_def && !piece_def->mesh_path.empty())
            {
                spawn.shooter.piece_mesh_path = piece_def->mesh_path;
                spawn.shooter.piece_name      = piece_def->name;
            }
            else
            {
                LOG_WARN("SceneLoader: shooter piece '%s' unresolved", piece_def_path.c_str());
            }
        }
        LOG_INFO("SceneLoader: robot shooter loaded  mesh='%s'",
                 spawn.shooter.piece_mesh_path.c_str());
    }
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
    try { f >> j; }
    catch (const json::parse_error &e)
    {
        LOG_ERROR("SceneLoader: parse error: %s", e.what());
        return result;
    }

    fs::path scene_dir  = fs::path(scene_path).parent_path();
    std::string scene_name = j.value("name", "unnamed");
    LOG_INFO("SceneLoader: loading scene '%s'", scene_name.c_str());

    // ── Non-robot bodies (field, game pieces) ─────────────────────────────
    std::unordered_map<std::string, BodyDef> def_cache;

    for (const auto &entry : j.value("bodies", json::array()))
    {
        if (!entry.contains("def"))
        {
            LOG_WARN("SceneLoader: body entry missing 'def', skipping");
            continue;
        }

        std::string def_path = (scene_dir / entry["def"].get<std::string>()).string();

        if (!def_cache.count(def_path))
        {
            auto maybe = LoadBodyDef(def_path, motors);
            if (!maybe)
            {
                LOG_WARN("SceneLoader: failed def: %s", def_path.c_str());
                continue;
            }
            def_cache[def_path] = std::move(*maybe);
        }

        SpawnRequest req;
        req.def         = def_cache[def_path];
        req.role        = entry.value("role", "");
        req.position    = readVec3(entry, "position");
        req.orientation = readVec4(entry, "orientation");

        result.bodies.push_back(std::move(req));
    }

    // ── Robot spawn points ────────────────────────────────────────────────
    // Each entry defines where a robot spawns and its optional mechanisms.
    // Indexed by --robot arg order.
    //
    // Scene JSON example:
    //   "robots": [
    //     {
    //       "position": [2.0, 0.051, 0.0],
    //       "orientation": [0,0,0,1],
    //       "intake": { "center": [...], "half_extents": [...], "max_capacity": 1 },
    //       "shooter": { "exit_point": [...], "piece_mesh_path": "...", "fire_rate": 2 }
    //     },
    //     {
    //       "position": [-2.0, 0.051, 0.0],
    //       "orientation": [0,1,0,0]
    //       // no mechanisms for robot 1
    //     }
    //   ]

    for (const auto &rs : j.value("robots", json::array()))
    {
        RobotSpawn spawn;
        spawn.position    = readVec3(rs, "position",    {0.f, 0.051f, 0.f});
        spawn.orientation = readVec4(rs, "orientation");
        ParseMechanisms(rs, scene_dir, motors, spawn);
        result.robot_spawns.push_back(std::move(spawn));
    }

    LOG_INFO("SceneLoader: %zu bodies, %zu robot spawns from '%s'",
             result.bodies.size(), result.robot_spawns.size(), scene_name.c_str());
    return result;
}