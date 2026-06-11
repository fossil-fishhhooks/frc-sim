#pragma once
#include "core/BodyDef.h"
#include "core/ScoreTracker.h"
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Per-motor state captured each snapshot
// ─────────────────────────────────────────────────────────────────────────────
struct MotorSnapshot
{
    float omega          = 0.f;
    float normal_force   = 0.f;
    float tractive_force = 0.f;
    bool  slipping       = false;

    float position[3]  = {};
    float direction[3] = {};
};

// ─────────────────────────────────────────────────────────────────────────────
// Per-body state
// ─────────────────────────────────────────────────────────────────────────────
struct BodySnapshot
{
    float pos[3] = {};
    float rot[4] = {};

    const BodyDef *def = nullptr;
    float vel[3] = {};  // linear velocity m/s — add after rot[4]

    std::vector<MotorSnapshot> motors;
};

// ─────────────────────────────────────────────────────────────────────────────
// Per-robot mechanism state — one entry per robot slot
// ─────────────────────────────────────────────────────────────────────────────
struct RobotMechSnapshot
{
    int  intake_held         = 0;
    int  intake_max_capacity = 0;
    bool shooter_armed       = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// Full world snapshot
// ─────────────────────────────────────────────────────────────────────────────
struct WorldSnapshot
{
    float sim_time = 0.f;

    std::vector<BodySnapshot> bodies;

    // robot_indices[i] = body index of robot i in bodies[]
    // robot_mech[i]    = mechanism state for robot i
    std::vector<int>               robot_indices;
    std::vector<RobotMechSnapshot> robot_mech;

    ScoreTracker::State score_state;
    std::vector<ScoringZoneDef> score_zones;
};