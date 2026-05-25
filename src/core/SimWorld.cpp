///////////////////////////////
// This file is written by AI. I dont know Jolt.
#include "core/SimWorld.h"
#include "physics/ShapeLoader.h"
#include "io/EasyLog.h"

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>

#include <thread>
#include <cstring>

// ── Jolt required callbacks ───────────────────────────────────────────────────

static void TraceImpl(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
}

#ifdef JPH_ENABLE_ASSERTS
static bool AssertFailed(const char *expr, const char *msg,
                         const char *file, JPH::uint line)
{
    LOG_WTF("Jolt assert failed: %s (%s) at %s:%u", expr, msg ? msg : "", file, line);
    return true;
}
#endif

// ── Layer interface implementations ──────────────────────────────────────────

struct SimWorld::BPLayerImpl : public JPH::BroadPhaseLayerInterface
{
    JPH::uint GetNumBroadPhaseLayers() const override
    {
        return BPLayers::NUM;
    }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override
    {
        return layer == Layers::STATIC ? BPLayers::STATIC : BPLayers::DYNAMIC;
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override
    {
        return layer == BPLayers::STATIC ? "STATIC" : "DYNAMIC";
    }
#endif
};

struct SimWorld::OBPLayerImpl : public JPH::ObjectVsBroadPhaseLayerFilter
{
    bool ShouldCollide(JPH::ObjectLayer obj, JPH::BroadPhaseLayer bp) const override
    {
        switch (obj)
        {
        case Layers::STATIC:
            return bp == BPLayers::DYNAMIC;
        case Layers::DYNAMIC:
            return true;
        default:
            return false;
        }
    }
};

struct SimWorld::OOLayerImpl : public JPH::ObjectLayerPairFilter
{
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override
    {
        // Static vs static: never collide
        if (a == Layers::STATIC && b == Layers::STATIC)
            return false;
        return true;
    }
};

// ── SimWorld ──────────────────────────────────────────────────────────────────

SimWorld::SimWorld() = default;
SimWorld::~SimWorld() = default;

void SimWorld::Init()
{
    // One-time Jolt process init
    JPH::RegisterDefaultAllocator();
    JPH::Trace = TraceImpl;
#ifdef JPH_ENABLE_ASSERTS
    JPH::AssertFailed = AssertFailed;
#endif
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    // Allocator: 32 MB temp scratch for Jolt's internal use each step
    m_temp_alloc = std::make_unique<JPH::TempAllocatorImpl>(32 * 1024 * 1024);

    // Job system: use all but one hardware thread
    int threads = std::max(1, (int)std::thread::hardware_concurrency() - 1);
    m_job_system = std::make_unique<JPH::JobSystemThreadPool>(
        JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, threads);

    // Layer interfaces
    m_bp_iface = std::make_unique<BPLayerImpl>();
    m_obj_vs_bp = std::make_unique<OBPLayerImpl>();
    m_obj_vs_obj = std::make_unique<OOLayerImpl>();

    // Physics system
    m_physics = std::make_unique<JPH::PhysicsSystem>();
    m_physics->Init(
        2048, // max bodies
        0,    // num body mutexes (0 = auto)
        4096, // max body pairs
        4096, // max contact constraints
        *m_bp_iface, *m_obj_vs_bp, *m_obj_vs_obj);

    m_physics->SetGravity(JPH::Vec3(0.0f, -9.81f, 0.0f));
    m_physics->SetContactListener(&m_contact_listener);

    LOG_INFO("SimWorld: initialized (threads=%d)", threads);
}

JPH::BodyID SimWorld::SpawnBody(const BodyDef &def,
                                const float pos[3],
                                const float rot[4])
{
    bool is_static = (def.mass == 0.0f);

    auto shape = LoadShape(def.mesh_path, is_static);
    if (!shape)
    {
        LOG_ERROR("SimWorld: SpawnBody failed for '%s' — no shape", def.name.c_str());
        return JPH::BodyID();
    }

    JPH::RVec3 jph_pos(pos[0], pos[1], pos[2]);
    JPH::Quat jph_rot(rot[0], rot[1], rot[2], rot[3]);

    auto motion = is_static ? JPH::EMotionType::Static
                            : JPH::EMotionType::Dynamic;
    auto layer = is_static ? Layers::STATIC : Layers::DYNAMIC;

    // Apply COM offset BEFORE constructing BodyCreationSettings.
    // OffsetCenterOfMassShape wraps the collision shape and shifts the COM.
    // Jolt then computes the inertia tensor about the correct COM when
    // CalculateInertia is used. The previous code applied the offset shape
    // AFTER constructing settings, so settings held the original shape and
    // the offset was completely ignored.
    if (!is_static)
    {
        JPH::Vec3 offset(def.com_offset[0], def.com_offset[1], def.com_offset[2]);
        if (!offset.IsNearZero())
        {
            JPH::OffsetCenterOfMassShapeSettings com_settings(offset, shape.GetPtr());
            auto result = com_settings.Create();
            if (result.HasError())
            {
                LOG_ERROR("SimWorld: OffsetCenterOfMassShape failed for '%s': %s",
                          def.name.c_str(), result.GetError().c_str());
            }
            else
            {
                shape = result.Get();
                LOG_DEBUG("SimWorld: COM offset (%.3f, %.3f, %.3f) applied to '%s'",
                          offset.GetX(), offset.GetY(), offset.GetZ(), def.name.c_str());
            }
        }
    }

    JPH::BodyCreationSettings settings(shape, jph_pos, jph_rot, motion, layer);

    if (!is_static)
    {
        settings.mMassPropertiesOverride.mMass = def.mass;
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    }

    // Dynamic bodies: zero Jolt friction — ForceApplicator
    // owns all drive and lateral friction via AddForce().
    // Letting Jolt also apply friction double-counts it:
    // straight driving roughly cancels out, but during turns
    // Jolt's constraint friction fights the drive forces and
    // makes the robot feel locked to the ground.
    // Static field bodies keep their friction normally so
    // game pieces that aren't motor-driven behave correctly.
    settings.mFriction = is_static ? def.surface.cof_static : 0.0f;
    settings.mRestitution = def.surface.restitution;

    auto &bi = m_physics->GetBodyInterface();
    JPH::Body *body = bi.CreateBody(settings);
    if (!body)
    {
        LOG_ERROR("SimWorld: Jolt body creation failed for '%s'", def.name.c_str());
        return JPH::BodyID();
    }

    bi.AddBody(body->GetID(), JPH::EActivation::Activate);

    // emplace_back constructs in-place — required because std::atomic is not movable.
    // deque guarantees no reallocation moves existing elements.
    m_bodies.emplace_back();
    BodyRecord &rec = m_bodies.back();
    rec.jph_id = body->GetID();
    rec.def = &def;
    for (int i = 0; i < BodyRecord::MAX_MOTORS; ++i)
    {
        rec.voltage[i].store(0.0f);
        rec.omega[i].store(0.0f);
        rec.normal_force[i].store(0.0f);
        rec.tractive_force[i].store(0.0f);
        rec.slipping[i].store(false);
    }

    LOG_INFO("SimWorld: spawned '%s' at (%.2f, %.2f, %.2f) [%s]",
             def.name.c_str(), pos[0], pos[1], pos[2],
             is_static ? "static" : "dynamic");

    return body->GetID();
}

void SimWorld::Step(float dt)
{
    constexpr int collision_steps = 1;
    m_physics->Update(dt, collision_steps, m_temp_alloc.get(), m_job_system.get());
    m_sim_time += dt;
}

void SimWorld::CaptureSnapshot(WorldSnapshot &out) const
{
    auto &bi = m_physics->GetBodyInterface();
    out.bodies.clear();
    out.bodies.reserve(m_bodies.size());
    out.sim_time = m_sim_time;
    out.robot_index = m_robot_index;

    for (int i = 0; i < (int)m_bodies.size(); ++i)
    {
        const auto &rec = m_bodies[i];

        // GetPosition() returns the body origin in world space — this is the
        // point the mesh was placed at when spawned, and what the renderer
        // needs to correctly place the mesh.
        //
        // The previous code used GetCenterOfMassPosition(), which differs from
        // the body origin by the COM offset. That caused the rendered mesh to
        // be visually offset from the collision shape whenever com_offset != 0.
        JPH::RVec3 pos = bi.GetPosition(rec.jph_id);
        JPH::Quat rot = bi.GetRotation(rec.jph_id);

        BodySnapshot snap;
        snap.pos[0] = (float)pos.GetX();
        snap.pos[1] = (float)pos.GetY();
        snap.pos[2] = (float)pos.GetZ();
        snap.rot[0] = rot.GetX();
        snap.rot[1] = rot.GetY();
        snap.rot[2] = rot.GetZ();
        snap.rot[3] = rot.GetW();
        snap.def = rec.def;

        // Motor snapshots — only populated for the robot body
        if (i == m_robot_index && rec.def)
        {
            int n = std::min((int)rec.def->motors.size(),
                             (int)BodyRecord::MAX_MOTORS);
            snap.motors.resize(n);
            for (int m = 0; m < n; ++m)
            {
                snap.motors[m].omega = rec.omega[m].load();
                snap.motors[m].normal_force = rec.normal_force[m].load();
                snap.motors[m].tractive_force = rec.tractive_force[m].load();
                snap.motors[m].slipping = rec.slipping[m].load();
                // position and direction are static from def (local space)
                const auto &att = rec.def->motors[m].attachment;
                const auto &dir = rec.def->motors[m].direction;
                snap.motors[m].position[0] = att[0];
                snap.motors[m].position[1] = att[1];
                snap.motors[m].position[2] = att[2];
                snap.motors[m].direction[0] = dir[0];
                snap.motors[m].direction[1] = dir[1];
                snap.motors[m].direction[2] = dir[2];
            }
        }

        out.bodies.push_back(std::move(snap));
    }
}

void SimWorld::SetMotorVoltage(int body_idx, int motor_idx, float voltage)
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size())
        return;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS)
        return;
    m_bodies[body_idx].voltage[motor_idx].store(voltage);
}

float SimWorld::GetMotorOmega(int body_idx, int motor_idx) const
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size())
        return 0.0f;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS)
        return 0.0f;
    return m_bodies[body_idx].omega[motor_idx].load();
}

void SimWorld::SetMotorOmega(int body_idx, int motor_idx, float omega)
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size())
        return;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS)
        return;
    m_bodies[body_idx].omega[motor_idx].store(omega);
}

void SimWorld::SetMotorNormalForce(int body_idx, int motor_idx, float force)
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size())
        return;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS)
        return;
    m_bodies[body_idx].normal_force[motor_idx].store(force);
}

void SimWorld::SetMotorTractiveForce(int body_idx, int motor_idx, float force)
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size())
        return;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS)
        return;
    m_bodies[body_idx].tractive_force[motor_idx].store(force);
}

void SimWorld::SetMotorSlipping(int body_idx, int motor_idx, bool slipping)
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size())
        return;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS)
        return;
    m_bodies[body_idx].slipping[motor_idx].store(slipping);
}

float SimWorld::GetMotorNormalForce(int body_idx, int motor_idx) const
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size())
        return 0.0f;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS)
        return 0.0f;
    return m_bodies[body_idx].normal_force[motor_idx].load();
}

float SimWorld::GetMotorTractiveForce(int body_idx, int motor_idx) const
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size())
        return 0.0f;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS)
        return 0.0f;
    return m_bodies[body_idx].tractive_force[motor_idx].load();
}

bool SimWorld::GetMotorSlipping(int body_idx, int motor_idx) const
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size())
        return false;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS)
        return false;
    return m_bodies[body_idx].slipping[motor_idx].load();
}

JPH::BodyInterface &SimWorld::GetBodyInterface()
{
    return m_physics->GetBodyInterface();
}

JPH::BodyID SimWorld::GetBodyID(int idx) const
{
    if (idx < 0 || idx >= (int)m_bodies.size())
        return JPH::BodyID();
    return m_bodies[idx].jph_id;
}

const BodyDef *SimWorld::GetBodyDef(int idx) const
{
    if (idx < 0 || idx >= (int)m_bodies.size())
        return nullptr;
    return m_bodies[idx].def;
}

float SimWorld::GetMotorVoltage(int body_idx, int motor_idx) const
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size())
        return 0.0f;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS)
        return 0.0f;
    return m_bodies[body_idx].voltage[motor_idx].load();
}
void SimWorld::MarkBodyRemoved(JPH::BodyID id)
{
    for (int i = 0; i < (int)m_bodies.size(); ++i)
    {
        if (m_bodies[i].jph_id != id)
            continue;

        int last = (int)m_bodies.size() - 1;

        if (i != last)
        {
            // If the last slot is the robot, its index moves to i
            if (m_robot_index == last)
                m_robot_index = i;

            // Swap dead slot with back — deque doesn't support move of atomics
            // so copy field by field
            m_bodies[i].jph_id = m_bodies[last].jph_id;
            m_bodies[i].def = m_bodies[last].def;
            for (int m = 0; m < BodyRecord::MAX_MOTORS; ++m)
            {
                m_bodies[i].voltage[m].store(m_bodies[last].voltage[m].load());
                m_bodies[i].omega[m].store(m_bodies[last].omega[m].load());
                m_bodies[i].normal_force[m].store(m_bodies[last].normal_force[m].load());
                m_bodies[i].tractive_force[m].store(m_bodies[last].tractive_force[m].load());
                m_bodies[i].slipping[m].store(m_bodies[last].slipping[m].load());
            }
        }
        else if (m_robot_index == last)
        {
            // Removing the robot itself??
            m_robot_index = -1;
        }

        m_bodies.pop_back();
        return;
    }
}
bool SimWorld::SpawnProjectile(const std::string &mesh_path, float mass,
                               const std::string &piece_name,
                               JPH::Vec3 world_pos, JPH::Vec3 world_vel)
{
    auto proj_def = std::make_unique<BodyDef>();
    proj_def->name = piece_name;
    proj_def->name = "projectile";
    proj_def->mesh_path = mesh_path;
    proj_def->mass = mass;
    // Default surface props; projectiles tumble freely.
    proj_def->surface.cof_static = 0.4f;
    proj_def->surface.cof_dynamic = 0.3f;
    proj_def->surface.restitution = 0.5f;

    float pos[3] = {world_pos.GetX(), world_pos.GetY(), world_pos.GetZ()};
    float rot[4] = {0.f, 0.f, 0.f, 1.f}; // identity orientation at spawn

    JPH::BodyID new_id = SpawnBody(*proj_def, pos, rot);
    if (new_id.IsInvalid())
    {
        LOG_ERROR("SimWorld::SpawnProjectile: SpawnBody failed for '%s'", mesh_path.c_str());
        return false;
    }

    // Apply launch velocity
    m_physics->GetBodyInterface().SetLinearVelocity(new_id, world_vel);

    // Transfer ownership of the def so the BodyRecord pointer stays valid.
    m_projectile_defs.push_back(std::move(proj_def));

    // Patch the just-added BodyRecord to point at the owned def
    // (SpawnBody stored the raw pointer to proj_def before the move — re-point it).
    m_bodies.back().def = m_projectile_defs.back().get();

    LOG_INFO("SimWorld::SpawnProjectile: spawned at (%.2f, %.2f, %.2f)  vel=(%.1f, %.1f, %.1f)",
             pos[0], pos[1], pos[2],
             world_vel.GetX(), world_vel.GetY(), world_vel.GetZ());
    return true;
}

JPH::AABox SimWorld::GetBodyAABB(JPH::BodyID id) const
{
    JPH::BodyLockRead lock(m_physics->GetBodyLockInterface(), id);
    if (!lock.Succeeded())
        return JPH::AABox();
    return lock.GetBody().GetWorldSpaceBounds();
}