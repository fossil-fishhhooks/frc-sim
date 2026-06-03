#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Geometry/AABox.h>

#include "core/BodyDef.h"
#include "core/Snapshot.h"
#include "physics/ContactListener.h"

#include <vector>
#include <deque>
#include <memory>
#include <string>

// Object layers
namespace Layers
{
    static constexpr JPH::ObjectLayer STATIC = 0;
    static constexpr JPH::ObjectLayer DYNAMIC = 1;
    static constexpr JPH::uint NUM = 2;
}

// Broad phase layers
namespace BPLayers
{
    static constexpr JPH::BroadPhaseLayer STATIC{0};
    static constexpr JPH::BroadPhaseLayer DYNAMIC{1};
    static constexpr JPH::uint NUM{2};
}

class SimWorld
{
public:
    SimWorld();
    ~SimWorld();

    // Must be called once before SpawnBody().
    // Initialises Jolt allocators, job system, physics system, gravity.
    void Init();

    void SetPhysicsDt(float dt) { m_contact_listener.SetDt(dt); }

    // Spawn a body from a definition at the given world transform.
    // pos: world position (m), rot: xyzw quaternion
    // Returns BodyID — invalid if spawn failed.
    JPH::BodyID SpawnBody(const BodyDef &def,
                          const float pos[3],
                          const float rot[4]);

    // Advance the simulation by dt seconds (one fixed step).
    void Step(float dt);

    // Write all body transforms + motor states into out_snapshot.
    // Called from SimLoop after Step().
    void CaptureSnapshot(WorldSnapshot &out_snapshot) const;

    // Set voltage command for a motor slot on a specific body.
    // motor_idx matches order in BodyDef::motors[].
    // Written by NTClient (IO thread), read by ForceApplicator (physics thread).
    void SetMotorVoltage(int body_idx, int motor_idx, float voltage);

    // Read current omega for a motor (updated by ForceApplicator each tick).
    float GetMotorOmega(int body_idx, int motor_idx) const;

    // Set omega — called by ForceApplicator after integration.
    void SetMotorOmega(int body_idx, int motor_idx, float omega);

    // Set normal force and tractive force for visualization (called by ForceApplicator).
    void SetMotorNormalForce(int body_idx, int motor_idx, float force);
    void SetMotorTractiveForce(int body_idx, int motor_idx, float force);
    void SetMotorSlipping(int body_idx, int motor_idx, bool slipping);

    float GetMotorNormalForce(int body_idx, int motor_idx) const;
    float GetMotorTractiveForce(int body_idx, int motor_idx) const;
    bool GetMotorSlipping(int body_idx, int motor_idx) const;

    // Number of spawned bodies.
    int BodyCount() const { return static_cast<int>(m_bodies.size()); }

    // Mark a body slot as removed (called by MechanismSystem after RemoveBody).
    // The BodyRecord's jph_id is invalidated so the renderer and ForceApplicator skip it.
    void MarkBodyRemoved(JPH::BodyID id);

    // Spawn a projectile (re-ejected game piece) at world_pos with world_vel.
    // Loads the mesh shape the same way SpawnBody does.
    // Returns true on success. Called from MechanismSystem::RunShooter().
    bool SpawnProjectile(const std::string &mesh_path, float mass,
                         const std::string &piece_name,
                         JPH::Vec3 world_pos, JPH::Vec3 world_vel);

    // Direct Jolt body interface — used by ForceApplicator.
    JPH::BodyInterface &GetBodyInterface();

    // World-space AABB for a body (via BodyLockRead — safe from physics thread).
    // Returns a zero-sized box at the origin if the ID is invalid.
    JPH::AABox GetBodyAABB(JPH::BodyID id) const;

    // Body ID for a spawned body by index.
    JPH::BodyID GetBodyID(int idx) const;

    // BodyDef for a body by index (nullptr if out of range).
    const BodyDef *GetBodyDef(int idx) const;

    // Read voltage — called by ForceApplicator (physics thread).
    float GetMotorVoltage(int body_idx, int motor_idx) const;

    static constexpr int MAX_MOTORS_PER_BODY = 8;

    // Index of the body marked as "robot" (-1 if none).
    int RobotIndex() const { return m_robot_index; }
    void SetRobotIndex(int idx) { m_robot_index = idx; }

    // Contact listener — pass to ForceApplicator.
    ContactListener &GetContactListener() { return m_contact_listener; }


    const JPH::BodyLockInterface &GetBodyLockInterface() const
    {
        return m_physics->GetBodyLockInterface();
    }

    void  SetMotorSteerAngle(int body_idx, int motor_idx, float radians);
    float GetMotorSteerAngle(int body_idx, int motor_idx) const;

private:
    ContactListener m_contact_listener;

    // Jolt subsystems
    std::unique_ptr<JPH::TempAllocatorImpl> m_temp_alloc;
    std::unique_ptr<JPH::JobSystemThreadPool> m_job_system;
    std::unique_ptr<JPH::PhysicsSystem> m_physics;

    // Layer interface implementations (defined in SimWorld.cpp)
    struct BPLayerImpl;
    struct OBPLayerImpl;
    struct OOLayerImpl;
    std::unique_ptr<BPLayerImpl> m_bp_iface;
    std::unique_ptr<OBPLayerImpl> m_obj_vs_bp;
    std::unique_ptr<OOLayerImpl> m_obj_vs_obj;

    

    // Per-body runtime record
    struct BodyRecord
    {
        JPH::BodyID jph_id;
        const BodyDef *def;

        // Per-motor runtime state — atomic floats for NT/physics thread safety
        // Max 8 motors per body (swerve has 8: 4 drive + 4 steer)
        static constexpr int MAX_MOTORS = 8;
        std::atomic<float> voltage[MAX_MOTORS];
        std::atomic<float> omega[MAX_MOTORS];
        std::atomic<float> normal_force[MAX_MOTORS];
        std::atomic<float> tractive_force[MAX_MOTORS];
        std::atomic<bool> slipping[MAX_MOTORS];
        std::atomic<float> steer_angle[MAX_MOTORS];  // radians, written by NT

        // Pre-allocated motor snapshot storage — sized once at SpawnBody,
        // never resized at runtime. Static fields pre-filled at spawn.
        std::vector<MotorSnapshot> motor_snap_cache;
    };

    // deque gives stable element addresses on growth — required because
    // BodyRecord contains std::atomic<float> which is not movable.
    std::deque<BodyRecord> m_bodies;
    // BodyDefs are owned by the caller (SceneLoader SpawnRequests).
    // SimWorld only holds raw pointers back to them for snapshots.

    int m_robot_index = -1;
    double m_sim_time = 0.0;

    // BodyDefs owned by SimWorld for dynamically spawned projectiles.
    // SpawnBody for scene bodies uses caller-owned defs (raw pointer);
    // projectiles need a stable def lifetime managed here.
    std::vector<std::unique_ptr<BodyDef>> m_projectile_defs;


    mutable WorldSnapshot m_snap_scratch;
    mutable bool          m_snap_scratch_ready = false;
};