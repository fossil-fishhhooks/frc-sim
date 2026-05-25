#pragma once
#include "core/SimWorld.h"
#include "core/MechanismDef.h"
#include <atomic>

// ─────────────────────────────────────────────────────────────────────────────
// MechanismSystem
//
// Runs once per physics tick (called from SimLoop / ForceApplicator step).
//
// INTAKE
//   Transforms the IntakeDef OBB into world space using the robot body's
//   current transform, then iterates every non-robot dynamic body.
//   For each candidate:
//     1. Get its world AABB from Jolt.
//     2. Transform the AABB centre into OBB local space.
//     3. Test all 3 axes: |local_centre[i]| + half_size[i] ≤ half_extents[i]
//        (conservative — uses AABB half-size as bounding sphere).
//     4. On success: DeactivateBody + RemoveBody from the physics world,
//        mark it as swallowed in a local set, increment m_held.
//
// SHOOTER
//   Fired when m_fire_pending is set (by NTClient from any thread).
//   Re-spawns one piece: AddBody with the ejected body's shape and mass,
//   SetLinearVelocity to m_fire_speed * normalise(m_fire_direction),
//   decrements m_held.
//   The re-spawned piece uses ShooterDef::piece_mesh_path so the renderer
//   picks it up on the next snapshot rebuild (SimWorld should tag the new
//   body with a BodyDef* whose mesh_path matches).
// ─────────────────────────────────────────────────────────────────────────────

class MechanismSystem
{
public:
    MechanismSystem(SimWorld &world,
                    const IntakeDef &intake,
                    const ShooterDef &shooter);

    // Called from the physics thread each tick.
    void Tick(float dt);

    // ── Thread-safe NT interface (called from main/NT thread) ─────────────

    // Arm a shot. Direction is robot-local (will be world-transformed at fire).
    // speed is m/s. Safe to call from any thread.
    void ArmShot(float speed, float dir_x, float dir_y, float dir_z);

    int HeldCount() const { return m_held.load(); }
    int IntakeCapacity() const { return m_intake.max_capacity; }
    bool IsFirePending() const { return m_fire_pending.load(); }

private:
    SimWorld &m_world;
    const IntakeDef &m_intake;
    const ShooterDef &m_shooter;

    std::atomic<int> m_held{0};

    // Pending fire state — set by ArmShot, consumed by Tick
    std::atomic<bool> m_fire_pending{false};
    // These are written by ArmShot (NT thread) and read by Tick (physics thread).
    // A simple store/load is fine: worst case we fire with one stale value,
    // which is acceptable for a game simulator.
    float m_fire_speed{10.f};
    float m_fire_dir[3]{0.f, 0.707f, 0.707f};

    void RunIntake();
    void RunShooter();
};