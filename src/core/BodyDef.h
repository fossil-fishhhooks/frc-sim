#pragma once
#include <string>
#include <vector>
#include <array>



struct SurfaceProps {
    float cof_static   = 0.6f;
    float cof_dynamic  = 0.4f;
    float restitution  = 0.2f;
};

struct WheelProps {
    float radius      = 0.0508f;  // meters (2 inch default)
    float cof_static  = 0.9f;
    float cof_dynamic = 0.6f;
};

struct MotorAttachmentDef {
    std::string          profile_name;              // "Kraken", "NEO", etc.
    float                gear_ratio   = 1.0f;
    std::array<float, 3> attachment   = {0,0,0};    // local offset from COM (m)
    std::array<float, 3> direction    = {0,0,1};    // local unit vector
    bool                 is_wheel     = false;
    WheelProps           wheel;                     // only valid if is_wheel
};

struct BodyDef {
    std::string   name;
    std::string   mesh_path;   // used for render
    std::string   collision_mesh_path;     // used for both physics

    // mass == 0.0f = infinite (static body, field walls etc.)
    float mass = 0.0f;
    std::array<float, 3> com_offset = {0, 0, 0};

    SurfaceProps  surface;
    std::vector<MotorAttachmentDef> motors;  // empty for field / game pieces
};
