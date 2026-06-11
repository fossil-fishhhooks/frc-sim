#include "physics/MechanismSystem.h"
#include "io/EasyLog.h"

#include <Jolt/Jolt.h>
#include <Jolt/Geometry/AABox.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/Body.h>

#include <cmath>
#include <vector>

static JPH::Vec3 QuatRotate(const float q[4], JPH::Vec3 v)
{
    JPH::Quat jq(q[0], q[1], q[2], q[3]);
    return jq * v;
}

static JPH::Vec3 QuatRotateInv(const float q[4], JPH::Vec3 v)
{
    JPH::Quat jq(q[0], q[1], q[2], q[3]);
    return jq.Conjugated() * v;
}

MechanismSystem::MechanismSystem(SimWorld &world,
                                 const IntakeDef &intake,
                                 const ShooterDef &shooter,
                                 int robot_body_index)
    : m_world(world), m_intake(intake), m_shooter(shooter)
{
    const auto &ri = world.GetRobotIndices();
    for (int i = 0; i < (int)ri.size(); ++i)
        if (ri[i] == robot_body_index) { m_robot_slot = i; break; }
    LOG_INFO("MechanismSystem: bound to body_idx=%d  intake_cap=%d",
             robot_body_index, intake.max_capacity);
}

void MechanismSystem::Reset() {
    m_held.store(0);
    m_fire_pending.store(false);
}

void MechanismSystem::Tick(float dt)
{
    RunIntake();
    if (m_fire_pending.load())
        RunShooter();
}

void MechanismSystem::RunIntake()
{
    if (m_held.load() >= m_intake.max_capacity)
        return;

    const auto &ri_vec = m_world.GetRobotIndices();
    if (m_robot_slot < 0 || m_robot_slot >= (int)ri_vec.size()) return;
    int robot_idx = ri_vec[m_robot_slot];
    if (robot_idx < 0)
        return;

    JPH::BodyID robot_id = m_world.GetBodyID(robot_idx);
    if (robot_id.IsInvalid())
        return;

    auto &bi = m_world.GetBodyInterface();

    JPH::RMat44 robot_xform = bi.GetWorldTransform(robot_id);
    JPH::Quat   robot_rot   = bi.GetRotation(robot_id);

    JPH::Quat obb_rot_local(m_intake.orientation[0], m_intake.orientation[1],
                            m_intake.orientation[2], m_intake.orientation[3]);
    JPH::Quat obb_rot_world = robot_rot * obb_rot_local;

    JPH::Vec3 local_centre(m_intake.center[0],
                           m_intake.center[1],
                           m_intake.center[2]);
    JPH::Vec3 world_centre = robot_xform * local_centre;

    const float *he = m_intake.half_extents.data();

    std::vector<JPH::BodyID> to_swallow;

    for (int i = 0; i < m_world.BodyCount(); ++i)
    {
        // Skip all robot bodies — not just this one
        bool is_robot = false;
        for (int ri : m_world.GetRobotIndices())
            if (i == ri) { is_robot = true; break; }
        if (is_robot) continue;

        JPH::BodyID bid = m_world.GetBodyID(i);
        if (bid.IsInvalid()) continue;

        if (bi.GetMotionType(bid) != JPH::EMotionType::Dynamic)
            continue;

        JPH::AABox aabb        = m_world.GetBodyAABB(bid);
        JPH::Vec3  piece_centre = aabb.GetCenter();
        JPH::Vec3  piece_half   = aabb.GetExtent();
        float      piece_radius = piece_half.Length();

        JPH::Vec3 rel   = piece_centre - world_centre;
        JPH::Vec3 local = obb_rot_world.Conjugated() * rel;

        bool inside = (std::abs(local.GetX()) + piece_radius <= he[0]) &&
                      (std::abs(local.GetY()) + piece_radius <= he[1]) &&
                      (std::abs(local.GetZ()) + piece_radius <= he[2]);

        if (inside)
            to_swallow.push_back(bid);

        if ((int)to_swallow.size() + m_held.load() >= m_intake.max_capacity)
            break;
    }

    for (JPH::BodyID bid : to_swallow)
    {
        if (m_held.load() >= m_intake.max_capacity)
            break;

        JPH::BodyID saved_id = bid;
        bi.RemoveBody(bid);
        bi.DestroyBody(bid);
        m_world.MarkBodyRemoved(saved_id);

        int held = m_held.fetch_add(1) + 1;
        LOG_INFO("Intake[robot%d]: swallowed body %u  held=%d/%d",
                 m_robot_slot, bid.GetIndex(), held, m_intake.max_capacity);
    }
}

void MechanismSystem::RunShooter()
{
    m_fire_pending.store(false);

    int held = m_held.load();
    if (held <= 0)
    {
        LOG_WARN("Shooter[robot%d]: fire requested but intake empty", m_robot_slot);
        return;
    }

    const auto &ri_vec = m_world.GetRobotIndices();
    if (m_robot_slot < 0 || m_robot_slot >= (int)ri_vec.size()) return;
    int robot_idx = ri_vec[m_robot_slot];
    if (robot_idx < 0) return;

    JPH::BodyID robot_id = m_world.GetBodyID(robot_idx);
    if (robot_id.IsInvalid()) return;

    auto &bi = m_world.GetBodyInterface();

    JPH::RMat44 robot_xform = bi.GetWorldTransform(robot_id);
    JPH::Quat   robot_rot   = bi.GetRotation(robot_id);

    JPH::Vec3 local_exit(m_shooter.exit_point[0],
                         m_shooter.exit_point[1],
                         m_shooter.exit_point[2]);
    JPH::Vec3 world_exit = robot_xform * local_exit;

    JPH::Vec3 local_dir(m_fire_dir[0], m_fire_dir[1], m_fire_dir[2]);
    if (local_dir.LengthSq() < 1e-6f)
        local_dir = JPH::Vec3(m_shooter.default_direction[0],
                              m_shooter.default_direction[1],
                              m_shooter.default_direction[2]);

    JPH::Vec3 world_dir = (robot_rot * local_dir).Normalized();
    JPH::Vec3 velocity  = world_dir * m_fire_speed + bi.GetLinearVelocity(robot_id);

    bool ok = m_world.SpawnProjectile(m_shooter.piece_mesh_path,
                                      m_shooter.piece_mass,
                                      m_shooter.piece_name,
                                      world_exit,
                                      velocity);
    if (ok)
    {
        m_held.fetch_sub(1);
        LOG_INFO("Shooter[robot%d]: fired  speed=%.1f m/s  held=%d",
                 m_robot_slot, m_fire_speed, m_held.load());
    }
    else
    {
        LOG_WARN("Shooter[robot%d]: SpawnProjectile failed", m_robot_slot);
        m_held.fetch_sub(1);
    }
}

void MechanismSystem::ArmShot(float speed, float dx, float dy, float dz)
{
    m_fire_speed  = speed;
    m_fire_dir[0] = dx;
    m_fire_dir[1] = dy;
    m_fire_dir[2] = dz;
    m_fire_pending.store(true);
}