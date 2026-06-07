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

namespace Layers {
    static constexpr JPH::ObjectLayer STATIC  = 0;
    static constexpr JPH::ObjectLayer DYNAMIC = 1;
    static constexpr JPH::uint        NUM     = 2;
}
namespace BPLayers {
    static constexpr JPH::BroadPhaseLayer STATIC {0};
    static constexpr JPH::BroadPhaseLayer DYNAMIC{1};
    static constexpr JPH::uint            NUM    {2};
}

class SimWorld
{
public:
    SimWorld();
    ~SimWorld();

    void Init();
    void SetPhysicsDt(float dt) { m_contact_listener.SetDt(dt); }

    JPH::BodyID SpawnBody(const BodyDef &def,
                          const float pos[3],
                          const float rot[4]);

    void Step(float dt);
    void CaptureSnapshot(WorldSnapshot &out) const;

    void  SetMotorVoltage      (int body_idx, int motor_idx, float voltage);
    float GetMotorOmega        (int body_idx, int motor_idx) const;
    void  SetMotorOmega        (int body_idx, int motor_idx, float omega);
    void  SetMotorNormalForce  (int body_idx, int motor_idx, float force);
    void  SetMotorTractiveForce(int body_idx, int motor_idx, float force);
    void  SetMotorSlipping     (int body_idx, int motor_idx, bool  slipping);
    float GetMotorNormalForce  (int body_idx, int motor_idx) const;
    float GetMotorTractiveForce(int body_idx, int motor_idx) const;
    bool  GetMotorSlipping     (int body_idx, int motor_idx) const;
    float GetMotorVoltage      (int body_idx, int motor_idx) const;
    void  SetMotorSteerAngle   (int body_idx, int motor_idx, float radians);
    float GetMotorSteerAngle   (int body_idx, int motor_idx) const;

    int BodyCount() const { return static_cast<int>(m_bodies.size()); }

    void MarkBodyRemoved(JPH::BodyID id);
    bool SpawnProjectile(const std::string &mesh_path, float mass,
                         const std::string &piece_name,
                         JPH::Vec3 world_pos, JPH::Vec3 world_vel);

    JPH::BodyInterface           &GetBodyInterface();
    const JPH::BodyLockInterface &GetBodyLockInterface() const;
    JPH::AABox     GetBodyAABB(JPH::BodyID id) const;
    JPH::BodyID    GetBodyID  (int idx) const;
    const BodyDef *GetBodyDef (int idx) const;

    static constexpr int MAX_MOTORS_PER_BODY = 8;

    // Register a body as a robot. Returns its robot slot index (0-based).
    int AddRobotIndex(int body_idx);
    const std::vector<int> &GetRobotIndices() const { return m_robot_indices; }

    // Returns first robot index for ForceApplicator logging compat
    int RobotIndex() const { return m_robot_indices.empty() ? -1 : m_robot_indices[0]; }

    ContactListener &GetContactListener() { return m_contact_listener; }


private:
    ContactListener m_contact_listener;

    std::unique_ptr<JPH::TempAllocatorImpl>   m_temp_alloc;
    std::unique_ptr<JPH::JobSystemThreadPool> m_job_system;
    std::unique_ptr<JPH::PhysicsSystem>       m_physics;

    struct BPLayerImpl;
    struct OBPLayerImpl;
    struct OOLayerImpl;
    std::unique_ptr<BPLayerImpl>  m_bp_iface;
    std::unique_ptr<OBPLayerImpl> m_obj_vs_bp;
    std::unique_ptr<OOLayerImpl>  m_obj_vs_obj;

    struct BodyRecord
    {
        JPH::BodyID     jph_id;
        const BodyDef  *def = nullptr;

        static constexpr int MAX_MOTORS = 8;
        std::atomic<float> voltage       [MAX_MOTORS];
        std::atomic<float> omega         [MAX_MOTORS];
        std::atomic<float> normal_force  [MAX_MOTORS];
        std::atomic<float> tractive_force[MAX_MOTORS];
        std::atomic<bool>  slipping      [MAX_MOTORS];
        std::atomic<float> steer_angle   [MAX_MOTORS];

        std::vector<MotorSnapshot> motor_snap_cache;
    };

    std::deque<BodyRecord> m_bodies;
    std::vector<int>       m_robot_indices;   // all robot body indices

    double m_sim_time = 0.0;

    mutable WorldSnapshot m_snap_scratch;
    mutable bool          m_snap_scratch_ready = false;

    std::vector<std::unique_ptr<BodyDef>> m_projectile_defs;
};