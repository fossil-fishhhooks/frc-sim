#pragma once
#include <array>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// IntakeDef — OBB in robot-local space.
//
// Any dynamic body whose bounding sphere fits entirely within the OBB is
// "swallowed": removed from Jolt and the intake counter is incremented (up to
// max_capacity). Checked once per physics tick.
// ─────────────────────────────────────────────────────────────────────────────
struct IntakeDef
{
    // Local-space centre of the OBB (metres, robot-body frame)
    std::array<float, 3> center       = { 0.f, 0.f, 0.f };

    // Half-extents along each local axis (metres)
    std::array<float, 3> half_extents = { 0.2f, 0.2f, 0.2f };

    // OBB orientation relative to the robot body (xyzw quaternion, default identity)
    std::array<float, 4> orientation  = { 0.f, 0.f, 0.f, 1.f };

    int max_capacity = 3;   // maximum game-pieces held simultaneously
};

// ─────────────────────────────────────────────────────────────────────────────
// ShooterDef — ejects a held piece back into the simulation.
//
// Exit point and default direction are robot-local. NT overrides angle & speed
// each fire pulse. The piece is re-spawned as a free Jolt body at the world-
// space exit with the computed velocity, and intake count is decremented.
// ─────────────────────────────────────────────────────────────────────────────
struct ShooterDef
{
    // Local-space exit point (metres, robot-body frame)
    std::array<float, 3> exit_point = { 0.f, 0.5f, 0.3f };

    // Local-space default launch direction (will be normalised at use-time).
    // NT can supply a different direction via /sim/shooter/direction (float[3]).
    std::array<float, 3> default_direction = { 0.f, 0.707f, 0.707f };

    // Asset used to re-spawn the ejected piece (matches the original piece mesh)
    std::string piece_mesh_path;
    std::string piece_name = "game_piece";

    // Mass of the re-spawned piece body (kg)
    float piece_mass = 0.25f;
};
