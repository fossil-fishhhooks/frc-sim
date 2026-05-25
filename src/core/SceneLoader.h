#pragma once
#include "core/BodyDef.h"
#include "core/MechanismDef.h"
#include "core/MotorRegistry.h"
#include <string>
#include <vector>
#include <array>

struct SpawnRequest
{
    BodyDef def;
    std::array<float, 3> position = {0, 0, 0};
    std::array<float, 4> orientation = {0, 0, 0, 1}; // xyzw quaternion
    std::string role;                                // "robot", "field", ""
};

struct SceneData
{
    std::vector<SpawnRequest> bodies;
    bool has_intake = false;
    IntakeDef intake;
    ShooterDef shooter;
};

// Loads scene JSON, resolves all body def paths, returns SceneData.
// Any failed body def is skipped with a warning.
// Returns empty SceneData if the scene file itself cannot be read.
SceneData LoadScene(const std::string &scene_path, const MotorRegistry &motors);