#include "core/BodyLoader.h"
#include "io/EasyLog.h"

#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

static std::array<float,3> readVec3(const json& j, const char* key,
                                    std::array<float,3> def = {0,0,0})
{
    if (!j.contains(key)) return def;
    const auto& v = j[key];
    return { v[0].get<float>(), v[1].get<float>(), v[2].get<float>() };
}

//Main

std::optional<BodyDef> LoadBodyDef(const std::string& json_path,
                                   const MotorRegistry& motors)
{
    std::ifstream f(json_path);
    if (!f.is_open()) {
        LOG_ERROR("BodyLoader: cannot open: %s", json_path.c_str());
        return std::nullopt;
    }

    json j;
    try { f >> j; }
    catch (const json::parse_error& e) {
        LOG_ERROR("BodyLoader: parse error in %s: %s", json_path.c_str(), e.what());
        return std::nullopt;
    }

    BodyDef def;
    def.name      = j.value("name", "unnamed");
    def.mesh_path = j.value("mesh", "");
    def.collision_mesh_path = j.value("collision_mesh", ""); // optional

    if (j.contains("mass")) {
        auto& m = j["mass"];
        def.mass = (m.is_string()) ? 0.0f : m.get<float>();
    }

    def.com_offset = readVec3(j, "com_offset");

    if (j.contains("surface")) {
        const auto& s   = j["surface"];
        def.surface.cof_static  = s.value("cof_static",  0.6f);
        def.surface.cof_dynamic = s.value("cof_dynamic", 0.4f);
        def.surface.restitution = s.value("restitution",  0.2f);
    }

    if (j.contains("motors")) {
        for (const auto& m : j["motors"]) {
            MotorAttachmentDef motor;
            motor.profile_name = m.value("profile", "NEO");
            motor.gear_ratio   = m.value("gear_ratio", 1.0f);
            motor.attachment   = readVec3(m, "attachment");
            motor.direction    = readVec3(m, "direction", {0,0,1});
            motor.is_wheel     = m.value("is_wheel", false);
            motor.is_steerable = m.value("is_steerable", false);

            if (!motors.Lookup(motor.profile_name))
                LOG_WARN("BodyLoader: unknown motor profile '%s' in %s",
                        motor.profile_name.c_str(), json_path.c_str());

            if (motor.is_wheel && m.contains("wheel")) {
                const auto& w      = m["wheel"];
                motor.wheel.radius      = w.value("radius",      0.0508f);
                motor.wheel.cof_static  = w.value("cof_static",  0.9f);
                motor.wheel.cof_dynamic = w.value("cof_dynamic", 0.6f);
            }

            def.motors.push_back(std::move(motor));
        }
    }

    LOG_INFO("BodyLoader: loaded '%s' from %s (%zu motors)",
             def.name.c_str(), json_path.c_str(), def.motors.size());
    return def;
}
