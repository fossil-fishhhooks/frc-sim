#pragma once
#include "io/EasyLog.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

// What a ray measures on hit.
// PIECE  — normalised distance to nearest game piece (dynamic, non-robot body)
// FIELD  — normalised distance to nearest static field geometry
// ANY    — nearest hit regardless of layer (min of both)
enum class RayTarget { PIECE, FIELD, ANY };

struct RayDef
{
    // Origin offset in robot-local space (metres).
    // +X = robot forward, +Z = robot left, +Y = up.
    float origin_offset[3] = {0.f, 0.f, 0.f};

    // Horizontal yaw relative to robot heading (degrees). 0 = straight ahead.
    float yaw_deg   = 0.f;

    // Vertical pitch downward from horizontal (degrees). 0 = flat, +ve = nose-down.
    float pitch_deg = 0.f;

    float      max_dist = 8.f;
    RayTarget  target   = RayTarget::ANY;

    // Render colour in the 3-D viewport (RGBA 0-255).
    uint8_t color[4] = {255, 255, 0, 200};
};

struct RaycastConfig
{
    // NT4 float[] topic — one value per ray, 0 = hit at origin, 1 = no hit / max_dist.
    std::string         nt_topic = "/sim/raycast/hits";

    // World-space Y for the ray origin before pitch is applied.
    float               origin_y = 0.15f;

    std::vector<RayDef> rays;
};

inline RayTarget ParseTarget(const std::string &s)
{
    if (s == "piece") return RayTarget::PIECE;
    if (s == "field") return RayTarget::FIELD;
    return RayTarget::ANY;
}

inline std::optional<RaycastConfig> LoadRaycastConfig(const std::string &path)
{
    try {
        std::ifstream f(path);
        if (!f) { LOG_ERROR("RaycastDef: cannot open %s", path.c_str()); return std::nullopt; }
        auto j = nlohmann::json::parse(f);

        RaycastConfig cfg;
        cfg.nt_topic = j.value("nt_topic",  cfg.nt_topic);
        cfg.origin_y = j.value("origin_y",  cfg.origin_y);

        for (auto &r : j.at("rays")) {
            RayDef rd;
            rd.yaw_deg   = r.value("yaw_deg",   0.f);
            rd.pitch_deg = r.value("pitch_deg",  0.f);
            rd.max_dist  = r.value("max_dist",   8.f);
            rd.target    = ParseTarget(r.value("target", "any"));

            auto off = r.value("origin_offset", std::vector<float>{0,0,0});
            if (off.size() >= 3) {
                rd.origin_offset[0]=off[0];
                rd.origin_offset[1]=off[1];
                rd.origin_offset[2]=off[2];
            }

            auto col = r.value("color", std::vector<int>{255,255,0,200});
            if (col.size() >= 4) {
                rd.color[0]=(uint8_t)col[0]; rd.color[1]=(uint8_t)col[1];
                rd.color[2]=(uint8_t)col[2]; rd.color[3]=(uint8_t)col[3];
            }

            cfg.rays.push_back(rd);
        }
        LOG_INFO("RaycastDef: loaded %zu rays from %s", cfg.rays.size(), path.c_str());
        return cfg;
    } catch (std::exception &e) {
        LOG_ERROR("RaycastDef: parse error: %s", e.what());
        return std::nullopt;
    }
}