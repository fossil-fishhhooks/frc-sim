#pragma once
#include "core/BodyDef.h"
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Per-motor state captured each snapshot
// ─────────────────────────────────────────────────────────────────────────────
struct MotorSnapshot
{
    float omega = 0.f;          // shaft speed (rad/s)
    float normal_force = 0.f;   // N
    float tractive_force = 0.f; // N (signed)
    bool slipping = false;

    float position[3] = {};  // local attachment point (m)
    float direction[3] = {}; // local drive direction (unit vec)
};

// ─────────────────────────────────────────────────────────────────────────────
// Per-body state
// ─────────────────────────────────────────────────────────────────────────────
struct BodySnapshot
{
    float pos[3] = {}; // world position
    float rot[4] = {}; // world rotation (xyzw quaternion)

    const BodyDef *def = nullptr; // pointer into scene data — never freed here

    std::vector<MotorSnapshot> motors;
};

// ─────────────────────────────────────────────────────────────────────────────
// Full world snapshot (lock-free copy taken by SimLoop at ~render rate)
// ─────────────────────────────────────────────────────────────────────────────
struct WorldSnapshot
{
    float sim_time = 0.f;
    int robot_index = -1;

    std::vector<BodySnapshot> bodies;

    // ── Mechanism state ───────────────────────────────────────────────────
    int intake_held = 0;         // pieces currently held
    int intake_max_capacity = 0; // max pieces (from scene def; 0 = no intake)
    bool shooter_armed = false;  // true while m_fire_pending is set
};