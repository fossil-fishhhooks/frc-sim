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
    JPH::uint GetNumBroadPhaseLayers() const override { return BPLayers::NUM; }
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
        switch (obj) {
        case Layers::STATIC:  return bp == BPLayers::DYNAMIC;
        case Layers::DYNAMIC: return true;
        default:              return false;
        }
    }
};

struct SimWorld::OOLayerImpl : public JPH::ObjectLayerPairFilter
{
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override
    {
        if (a == Layers::STATIC && b == Layers::STATIC) return false;
        return true;
    }
};

// ── SimWorld ──────────────────────────────────────────────────────────────────

SimWorld::SimWorld() = default;
SimWorld::~SimWorld() = default;

void SimWorld::Init()
{
    JPH::RegisterDefaultAllocator();
    JPH::Trace = TraceImpl;
#ifdef JPH_ENABLE_ASSERTS
    JPH::AssertFailed = AssertFailed;
#endif
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    m_temp_alloc = std::make_unique<JPH::TempAllocatorImpl>(32 * 1024 * 1024);

    int threads = std::max(1, (int)std::thread::hardware_concurrency() - 1);
    m_job_system = std::make_unique<JPH::JobSystemThreadPool>(
        JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, threads);

    m_bp_iface   = std::make_unique<BPLayerImpl>();
    m_obj_vs_bp  = std::make_unique<OBPLayerImpl>();
    m_obj_vs_obj = std::make_unique<OOLayerImpl>();

    m_physics = std::make_unique<JPH::PhysicsSystem>();
    m_physics->Init(
        4096, 0, 8192, 8192,
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

    const std::string &shape_path = def.collision_mesh_path.empty()
                                        ? def.mesh_path
                                        : def.collision_mesh_path;
    auto shape = LoadShape(shape_path, is_static);
    if (!shape) {
        LOG_ERROR("SimWorld: SpawnBody failed for '%s' — no shape", def.name.c_str());
        return JPH::BodyID();
    }

    JPH::RVec3 jph_pos(pos[0], pos[1], pos[2]);
    JPH::Quat  jph_rot(rot[0], rot[1], rot[2], rot[3]);

    auto motion = is_static ? JPH::EMotionType::Static  : JPH::EMotionType::Dynamic;
    auto layer  = is_static ? Layers::STATIC             : Layers::DYNAMIC;

    if (!is_static) {
        JPH::Vec3 offset(def.com_offset[0], def.com_offset[1], def.com_offset[2]);
        if (!offset.IsNearZero()) {
            JPH::OffsetCenterOfMassShapeSettings com_settings(offset, shape.GetPtr());
            auto result = com_settings.Create();
            if (result.HasError())
                LOG_ERROR("SimWorld: OffsetCenterOfMassShape failed for '%s': %s",
                          def.name.c_str(), result.GetError().c_str());
            else {
                shape = result.Get();
                LOG_DEBUG("SimWorld: COM offset (%.3f,%.3f,%.3f) applied to '%s'",
                          offset.GetX(), offset.GetY(), offset.GetZ(), def.name.c_str());
            }
        }
    }

    JPH::BodyCreationSettings settings(shape, jph_pos, jph_rot, motion, layer);

    if (!is_static) {
        settings.mMassPropertiesOverride.mMass = def.mass;
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    }

    bool has_motors         = !def.motors.empty();
    settings.mFriction      = (is_static || !has_motors) ? def.surface.cof_dynamic : 0.0f;
    settings.mRestitution   = def.surface.restitution;

    if (!is_static) {
        settings.mLinearDamping  = 0.15f;
        settings.mAngularDamping = 0.20f;
    }

    auto &bi = m_physics->GetBodyInterface();
    JPH::Body *body = bi.CreateBody(settings);
    if (!body) {
        LOG_ERROR("SimWorld: Jolt body creation failed for '%s'", def.name.c_str());
        return JPH::BodyID();
    }

    bi.AddBody(body->GetID(), JPH::EActivation::Activate);

    m_bodies.emplace_back();
    BodyRecord &rec = m_bodies.back();
    rec.jph_id = body->GetID();
    rec.def    = &def;

    for (int i = 0; i < BodyRecord::MAX_MOTORS; ++i) {
        rec.voltage[i].store(0.0f);
        rec.omega[i].store(0.0f);
        rec.normal_force[i].store(0.0f);
        rec.tractive_force[i].store(0.0f);
        rec.slipping[i].store(false);
        rec.steer_angle[i].store(0.0f);
    }

    // ── Pre-allocate motor snapshot cache ─────────────────────────────────
    // Sized once here so CaptureSnapshot never calls resize() at runtime.
    int n_motors = std::min((int)def.motors.size(), BodyRecord::MAX_MOTORS);
    rec.motor_snap_cache.resize(n_motors);

    // Pre-fill static fields (attachment + direction never change)
    for (int m = 0; m < n_motors; ++m) {
        const auto &att = def.motors[m].attachment;
        const auto &dir = def.motors[m].direction;
        rec.motor_snap_cache[m].position[0]  = att[0];
        rec.motor_snap_cache[m].position[1]  = att[1];
        rec.motor_snap_cache[m].position[2]  = att[2];
        rec.motor_snap_cache[m].direction[0] = dir[0];
        rec.motor_snap_cache[m].direction[1] = dir[1];
        rec.motor_snap_cache[m].direction[2] = dir[2];
    }

    // Invalidate scratch so next CaptureSnapshot rebuilds body list size
    m_snap_scratch_ready = false;

    LOG_INFO("SimWorld: spawned '%s' at (%.2f, %.2f, %.2f) [%s]",
             def.name.c_str(), pos[0], pos[1], pos[2],
             is_static ? "static" : "dynamic");

    return body->GetID();
}

void SimWorld::Step(float dt)
{
    constexpr int collision_steps = 2;
    m_physics->Update(dt, collision_steps, m_temp_alloc.get(), m_job_system.get());
    m_sim_time += dt;
}

void SimWorld::CaptureSnapshot(WorldSnapshot &out) const
{
    auto &bi = m_physics->GetBodyInterface();

    // ── One-time scratch setup ────────────────────────────────────────────
    // Reserve bodies vector to exact body count — no realloc after warmup.
    if (!m_snap_scratch_ready) {
        m_snap_scratch.bodies.clear();
        m_snap_scratch.bodies.reserve(m_bodies.size());
        // Pre-populate one BodySnapshot per slot with correctly-sized motor vec
        for (int i = 0; i < (int)m_bodies.size(); ++i) {
            BodySnapshot bs;
            bs.def = m_bodies[i].def;
            // Motors only for robot body — pre-size from cache
            if (i == m_robot_index && m_bodies[i].def)
                bs.motors.resize(m_bodies[i].motor_snap_cache.size());
            m_snap_scratch.bodies.push_back(std::move(bs));
        }
        m_snap_scratch_ready = true;
    }

    m_snap_scratch.sim_time    = m_sim_time;
    m_snap_scratch.robot_index = m_robot_index;

    // ── Per-body update ───────────────────────────────────────────────────
    // Resize if bodies were added/removed since last setup
    if (m_snap_scratch.bodies.size() != m_bodies.size()) {
        m_snap_scratch_ready = false;
        CaptureSnapshot(out);   // recurse once to rebuild
        return;
    }

    for (int i = 0; i < (int)m_bodies.size(); ++i)
    {
        const BodyRecord &rec = m_bodies[i];
        BodySnapshot     &bs  = m_snap_scratch.bodies[i];

        if (rec.jph_id.IsInvalid()) {
            bs.def = nullptr;
            continue;
        }

        // ── Bundle all Jolt reads into one body lock ──────────────────────
        // One lock acquisition vs the previous 2 separate bi.Get* calls.
        // Safe: CaptureSnapshot runs on the SimLoop thread between Steps.
        {
            JPH::BodyLockRead lock(m_physics->GetBodyLockInterface(), rec.jph_id);
            if (!lock.Succeeded()) { bs.def = nullptr; continue; }
            const JPH::Body &body = lock.GetBody();

            JPH::RVec3 pos = body.GetPosition();
            JPH::Quat  rot = body.GetRotation();

            bs.pos[0] = (float)pos.GetX();
            bs.pos[1] = (float)pos.GetY();
            bs.pos[2] = (float)pos.GetZ();
            bs.rot[0] = rot.GetX();
            bs.rot[1] = rot.GetY();
            bs.rot[2] = rot.GetZ();
            bs.rot[3] = rot.GetW();
        }
        bs.def = rec.def;

        // ── Motor state — robot only ──────────────────────────────────────
        if (i == m_robot_index && rec.def)
        {
            int n = (int)rec.motor_snap_cache.size();
            // motors vec already correctly sized — no resize() call
            bs.motors.resize(n);  // no-op if already correct size

            for (int m = 0; m < n; ++m) {
                MotorSnapshot &ms = bs.motors[m];
                // Dynamic fields — updated each tick
                ms.omega          = rec.omega[m].load(std::memory_order_relaxed);
                ms.normal_force   = rec.normal_force[m].load(std::memory_order_relaxed);
                ms.tractive_force = rec.tractive_force[m].load(std::memory_order_relaxed);
                ms.slipping       = rec.slipping[m].load(std::memory_order_relaxed);
                // Static fields — copied from pre-filled cache (no def dereference)
                ms.position[0]  = rec.motor_snap_cache[m].position[0];
                ms.position[1]  = rec.motor_snap_cache[m].position[1];
                ms.position[2]  = rec.motor_snap_cache[m].position[2];
                ms.direction[0] = rec.motor_snap_cache[m].direction[0];
                ms.direction[1] = rec.motor_snap_cache[m].direction[1];
                ms.direction[2] = rec.motor_snap_cache[m].direction[2];
            }
        }
    }

    out = m_snap_scratch;
}

// ── Motor accessors ───────────────────────────────────────────────────────────

void SimWorld::SetMotorVoltage(int body_idx, int motor_idx, float voltage)
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size()) return;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS) return;
    m_bodies[body_idx].voltage[motor_idx].store(voltage);
}

float SimWorld::GetMotorOmega(int body_idx, int motor_idx) const
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size()) return 0.0f;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS) return 0.0f;
    return m_bodies[body_idx].omega[motor_idx].load();
}

void SimWorld::SetMotorOmega(int body_idx, int motor_idx, float omega)
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size()) return;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS) return;
    m_bodies[body_idx].omega[motor_idx].store(omega);
}

void SimWorld::SetMotorNormalForce(int body_idx, int motor_idx, float force)
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size()) return;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS) return;
    m_bodies[body_idx].normal_force[motor_idx].store(force);
}

void SimWorld::SetMotorTractiveForce(int body_idx, int motor_idx, float force)
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size()) return;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS) return;
    m_bodies[body_idx].tractive_force[motor_idx].store(force);
}

void SimWorld::SetMotorSlipping(int body_idx, int motor_idx, bool slipping)
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size()) return;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS) return;
    m_bodies[body_idx].slipping[motor_idx].store(slipping);
}

float SimWorld::GetMotorNormalForce(int body_idx, int motor_idx) const
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size()) return 0.0f;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS) return 0.0f;
    return m_bodies[body_idx].normal_force[motor_idx].load();
}

float SimWorld::GetMotorTractiveForce(int body_idx, int motor_idx) const
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size()) return 0.0f;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS) return 0.0f;
    return m_bodies[body_idx].tractive_force[motor_idx].load();
}

bool SimWorld::GetMotorSlipping(int body_idx, int motor_idx) const
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size()) return false;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS) return false;
    return m_bodies[body_idx].slipping[motor_idx].load();
}

float SimWorld::GetMotorVoltage(int body_idx, int motor_idx) const
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size()) return 0.0f;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS) return 0.0f;
    return m_bodies[body_idx].voltage[motor_idx].load();
}

// ── Body management ───────────────────────────────────────────────────────────

JPH::BodyInterface &SimWorld::GetBodyInterface()
{
    return m_physics->GetBodyInterface();
}

JPH::BodyID SimWorld::GetBodyID(int idx) const
{
    if (idx < 0 || idx >= (int)m_bodies.size()) return JPH::BodyID();
    return m_bodies[idx].jph_id;
}

const BodyDef *SimWorld::GetBodyDef(int idx) const
{
    if (idx < 0 || idx >= (int)m_bodies.size()) return nullptr;
    return m_bodies[idx].def;
}

JPH::AABox SimWorld::GetBodyAABB(JPH::BodyID id) const
{
    JPH::BodyLockRead lock(m_physics->GetBodyLockInterface(), id);
    if (!lock.Succeeded()) return JPH::AABox();
    return lock.GetBody().GetWorldSpaceBounds();
}

void SimWorld::MarkBodyRemoved(JPH::BodyID id)
{
    for (int i = 0; i < (int)m_bodies.size(); ++i)
    {
        if (m_bodies[i].jph_id != id) continue;

        int last = (int)m_bodies.size() - 1;

        if (i != last) {
            if (m_robot_index == last) m_robot_index = i;

            m_bodies[i].jph_id = m_bodies[last].jph_id;
            m_bodies[i].def    = m_bodies[last].def;
            m_bodies[i].motor_snap_cache = std::move(m_bodies[last].motor_snap_cache);
            for (int m = 0; m < BodyRecord::MAX_MOTORS; ++m) {
                m_bodies[i].voltage[m].store(m_bodies[last].voltage[m].load());
                m_bodies[i].omega[m].store(m_bodies[last].omega[m].load());
                m_bodies[i].normal_force[m].store(m_bodies[last].normal_force[m].load());
                m_bodies[i].tractive_force[m].store(m_bodies[last].tractive_force[m].load());
                m_bodies[i].slipping[m].store(m_bodies[last].slipping[m].load());
            }
        } else if (m_robot_index == last) {
            m_robot_index = -1;
        }

        m_bodies.pop_back();
        m_snap_scratch_ready = false;  // body count changed — rebuild scratch
        return;
    }
}

bool SimWorld::SpawnProjectile(const std::string &mesh_path, float mass,
                               const std::string &piece_name,
                               JPH::Vec3 world_pos, JPH::Vec3 world_vel)
{
    auto proj_def = std::make_unique<BodyDef>();
    proj_def->name      = piece_name;
    proj_def->mesh_path = mesh_path;
    proj_def->mass      = mass;
    proj_def->surface.cof_static   = 0.4f;
    proj_def->surface.cof_dynamic  = 0.3f;
    proj_def->surface.restitution  = 0.5f;

    float pos[3] = { world_pos.GetX(), world_pos.GetY(), world_pos.GetZ() };
    float rot[4] = { 0.f, 0.f, 0.f, 1.f };

    JPH::BodyID new_id = SpawnBody(*proj_def, pos, rot);
    if (new_id.IsInvalid()) {
        LOG_ERROR("SimWorld::SpawnProjectile: SpawnBody failed for '%s'", mesh_path.c_str());
        return false;
    }

    m_physics->GetBodyInterface().SetLinearVelocity(new_id, world_vel);

    m_projectile_defs.push_back(std::move(proj_def));
    m_bodies.back().def = m_projectile_defs.back().get();

    LOG_INFO("SimWorld::SpawnProjectile: spawned at (%.2f,%.2f,%.2f) vel=(%.1f,%.1f,%.1f)",
             pos[0], pos[1], pos[2],
             world_vel.GetX(), world_vel.GetY(), world_vel.GetZ());
    return true;
}

void SimWorld::SetMotorSteerAngle(int body_idx, int motor_idx, float radians)
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size()) return;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS) return;
    m_bodies[body_idx].steer_angle[motor_idx].store(radians);
}

float SimWorld::GetMotorSteerAngle(int body_idx, int motor_idx) const
{
    if (body_idx < 0 || body_idx >= (int)m_bodies.size()) return 0.0f;
    if (motor_idx < 0 || motor_idx >= BodyRecord::MAX_MOTORS) return 0.0f;
    return m_bodies[body_idx].steer_angle[motor_idx].load();
}