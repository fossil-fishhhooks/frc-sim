#pragma once
#include "core/BodyDef.h"
#include "core/MechanismDef.h"
#include "core/MotorRegistry.h"
#include <string>
#include <vector>
#include <array>

struct SpawnRequest
{
    BodyDef              def;
    std::array<float, 3> position    = {0, 0, 0};
    std::array<float, 4> orientation = {0, 0, 0, 1};
    std::string          role;        // "field", "game_piece", etc — NOT "robot"
};

// Spawn point for a robot, defined in scene JSON.
// Indexed by robot slot order (matches --robot arg order).
struct RobotSpawn
{
    std::array<float, 3> position    = {0.f, 0.051f, 0.f};
    std::array<float, 4> orientation = {0.f, 0.f, 0.f, 1.f};

    // Optional per-robot mechanisms — each robot may have its own or none
    bool       has_mechanisms = false;
    IntakeDef  intake;
    ShooterDef shooter;
};

struct SceneData
{
    std::vector<SpawnRequest> bodies;       // field + game pieces only
    std::vector<RobotSpawn>   robot_spawns; // one per robot slot
};

SceneData LoadScene(const std::string &scene_path, const MotorRegistry &motors);