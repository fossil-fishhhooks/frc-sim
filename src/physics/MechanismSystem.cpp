#include "physics/MechanismSystem.h"
#include "io/EasyLog.h"

#include <Jolt/Jolt.h>
#include <Jolt/Geometry/AABox.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/Body.h>

#include <cmath>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

// Rotate a vector by a quaternion (xyzw storage)
static JPH::Vec3 QuatRotate(const float q[4], JPH::Vec3 v)
{
    // q = (x,y,z,w)
    JPH::Quat jq(q[0], q[1], q[2], q[3]);
    return jq * v;
}

// Inverse-rotate: apply conjugate
static JPH::Vec3 QuatRotateInv(const float q[4], JPH::Vec3 v)
{
    JPH::Quat jq(q[0], q[1], q[2], q[3]);
    return jq.Conjugated() * v;
}

// ─────────────────────────────────────────────────────────────────────────────
// MechanismSystem
// ─────────────────────────────────────────────────────────────────────────────

MechanismSystem::MechanismSystem(SimWorld &world,
                                 const IntakeDef &intake,
                                 const ShooterDef &shooter)
    : m_world(world), m_intake(intake), m_shooter(shooter)
{
}

void MechanismSystem::Tick(float dt)
{
    RunIntake();
    if (m_fire_pending.load())
        RunShooter();
}

// ─────────────────────────────────────────────────────────────────────────────
// INTAKE
// ─────────────────────────────────────────────────────────────────────────────
void MechanismSystem::RunIntake()
{
    if (m_held.load() >= m_intake.max_capacity)
        return;

    int robot_idx = m_world.RobotIndex();
    if (robot_idx < 0)
        return;

    JPH::BodyID robot_id = m_world.GetBodyID(robot_idx);
    if (robot_id.IsInvalid())
        return;

    auto &bi = m_world.GetBodyInterface();

    // ── Build world-space OBB ─────────────────────────────────────────────
    // Robot world transform
    JPH::RMat44 robot_xform = bi.GetWorldTransform(robot_id);
    // Robot rotation as quaternion for the OBB orientation
    JPH::Quat robot_rot = bi.GetRotation(robot_id);

    // Combined OBB orientation: robot rotation * OBB local orientation
    JPH::Quat obb_rot_local(m_intake.orientation[0], m_intake.orientation[1],
                            m_intake.orientation[2], m_intake.orientation[3]);
    JPH::Quat obb_rot_world = robot_rot * obb_rot_local;

    // OBB centre in world space
    JPH::Vec3 local_centre(m_intake.center[0],
                           m_intake.center[1],
                           m_intake.center[2]);
    JPH::Vec3 world_centre = robot_xform * local_centre;

    const float *he = m_intake.half_extents.data(); // half-extents

    // ── Test every non-robot dynamic body ─────────────────────────────────
    // Collect candidates first (avoid mutating while iterating)
    std::vector<JPH::BodyID> to_swallow;

    for (int i = 0; i < m_world.BodyCount(); ++i)
    {
        if (i == robot_idx)
            continue;

        JPH::BodyID bid = m_world.GetBodyID(i);
        if (bid.IsInvalid())
            continue;

        // Only dynamic (non-static) bodies can be game pieces
        if (bi.GetMotionType(bid) != JPH::EMotionType::Dynamic)
            continue;

        // World AABB of the candidate
        JPH::AABox aabb = m_world.GetBodyAABB(bid);
        JPH::Vec3 piece_centre = aabb.GetCenter();
        JPH::Vec3 piece_half = aabb.GetExtent();  // already half-sizes
        float piece_radius = piece_half.Length(); // conservative sphere

        // Transform piece centre into OBB local space
        JPH::Vec3 rel = piece_centre - world_centre;
        JPH::Vec3 local = obb_rot_world.Conjugated() * rel;

        // OBB containment test: the piece's bounding sphere must fit inside
        bool inside = (std::abs(local.GetX()) + piece_radius <= he[0]) &&
                      (std::abs(local.GetY()) + piece_radius <= he[1]) &&
                      (std::abs(local.GetZ()) + piece_radius <= he[2]);

        if (inside)
            to_swallow.push_back(bid);

        if ((int)to_swallow.size() + m_held.load() >= m_intake.max_capacity)
            break;
    }

    // ── Swallow ───────────────────────────────────────────────────────────
    for (JPH::BodyID bid : to_swallow)
    {
        if (m_held.load() >= m_intake.max_capacity)
            break;

        // Remove from Jolt, then destroy, then mark the SimWorld slot invalid
        JPH::BodyID saved_id = bid; // copy before destroy
        bi.RemoveBody(bid);
        bi.DestroyBody(bid);
        m_world.MarkBodyRemoved(saved_id); // use saved copy

        int held = m_held.fetch_add(1) + 1;
        LOG_INFO("Intake: swallowed body %u  held=%d/%d",
                 bid.GetIndex(), held, m_intake.max_capacity);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SHOOTER
// ─────────────────────────────────────────────────────────────────────────────
void MechanismSystem::RunShooter()
{
    m_fire_pending.store(false); // consume the request

    int held = m_held.load();
    if (held <= 0)
    {
        LOG_WARN("Shooter: fire requested but intake empty");
        return;
    }

    int robot_idx = m_world.RobotIndex();
    if (robot_idx < 0)
        return;

    JPH::BodyID robot_id = m_world.GetBodyID(robot_idx);
    if (robot_id.IsInvalid())
        return;

    auto &bi = m_world.GetBodyInterface();

    JPH::RMat44 robot_xform = bi.GetWorldTransform(robot_id);
    JPH::Quat robot_rot = bi.GetRotation(robot_id);

    // World-space exit position
    JPH::Vec3 local_exit(m_shooter.exit_point[0],
                         m_shooter.exit_point[1],
                         m_shooter.exit_point[2]);
    JPH::Vec3 world_exit = robot_xform * local_exit;

    // World-space launch direction
    JPH::Vec3 local_dir(m_fire_dir[0], m_fire_dir[1], m_fire_dir[2]);
    if (local_dir.LengthSq() < 1e-6f)
        local_dir = JPH::Vec3(m_shooter.default_direction[0],
                              m_shooter.default_direction[1],
                              m_shooter.default_direction[2]);

    JPH::Vec3 world_dir = (robot_rot * local_dir).Normalized();
    JPH::Vec3 velocity = world_dir * m_fire_speed;

    // Add robot's own velocity so the piece isn't instantly re-ingested
    velocity = velocity + bi.GetLinearVelocity(robot_id);

    // ── Re-spawn piece body ───────────────────────────────────────────────
    // Delegate to SimWorld which knows how to build a shape + body from a def
    bool ok = m_world.SpawnProjectile(m_shooter.piece_mesh_path,
                                      m_shooter.piece_mass,
                                      m_shooter.piece_name,
                                      world_exit,
                                      velocity);
    if (ok)
    {
        m_held.fetch_sub(1);
        LOG_INFO("Shooter: fired piece  speed=%.1f m/s  held now=%d",
                 m_fire_speed, m_held.load());
    }
    else
    {
        LOG_WARN("Shooter: SpawnProjectile failed — piece lost");
        m_held.fetch_sub(1); // still consume so we don't get stuck
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// NT interface
// ─────────────────────────────────────────────────────────────────────────────

void MechanismSystem::ArmShot(float speed, float dx, float dy, float dz)
{
    m_fire_speed = speed;
    m_fire_dir[0] = dx;
    m_fire_dir[1] = dy;
    m_fire_dir[2] = dz;
    m_fire_pending.store(true);
}