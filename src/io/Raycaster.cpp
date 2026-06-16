#include <Jolt/Jolt.h>   // ← must be first, before any other Jolt header

#include "io/Raycaster.h"
#include "core/SimWorld.h"
#include "io/EasyLog.h"

#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>

// ── Layer filters ─────────────────────────────────────────────────────────────

struct BPFilterStatic : public JPH::BroadPhaseLayerFilter {
    bool ShouldCollide(JPH::BroadPhaseLayer l) const override {
        return l == BPLayers::STATIC;
    }
};
struct BPFilterDynamic : public JPH::BroadPhaseLayerFilter {
    bool ShouldCollide(JPH::BroadPhaseLayer l) const override {
        return l == BPLayers::DYNAMIC;
    }
};
struct BPFilterAll : public JPH::BroadPhaseLayerFilter {
    bool ShouldCollide(JPH::BroadPhaseLayer) const override { return true; }
};
struct ObjFilterAll : public JPH::ObjectLayerFilter {
    bool ShouldCollide(JPH::ObjectLayer) const override { return true; }
};

// ── Raycaster ─────────────────────────────────────────────────────────────────

void Raycaster::Init(const RaycastConfig &cfg,
                     nt::NetworkTableInstance &inst,
                     int robot_slot)
{
    m_cfg        = &cfg;
    m_robot_slot = robot_slot;

    std::string topic = cfg.nt_topic;
    if (robot_slot > 0)
        topic += "/robot" + std::to_string(robot_slot);

    m_pub = inst.GetFloatArrayTopic(topic).Publish();
    m_hits.resize(cfg.rays.size(), 1.0f);
    m_render.resize(cfg.rays.size());
    LOG_INFO("Raycaster[%d]: %zu rays -> %s", robot_slot, cfg.rays.size(), topic.c_str());
}

void Raycaster::CastAndPublish(const WorldSnapshot &snapshot, SimWorld &world)
{


    if (!m_cfg) { LOG_INFO("Raycaster: no cfg"); return; }

    const auto &ri_vec = world.GetRobotIndices();
    //LOG_INFO("Raycaster: robot_slot=%d ri_vec.size=%d bodies=%d",
            // m_robot_slot, (int)ri_vec.size(), (int)snapshot.bodies.size());

    if (m_robot_slot >= (int)ri_vec.size()) { LOG_INFO("Raycaster: slot OOB"); return; }
    int robot_idx = ri_vec[m_robot_slot];
    if (robot_idx >= (int)snapshot.bodies.size()) { LOG_INFO("Raycaster: robot_idx OOB"); return; }


    const BodySnapshot &robot = snapshot.bodies[robot_idx];

    float qx=robot.rot[0], qy=robot.rot[1], qz=robot.rot[2], qw=robot.rot[3];
    float yaw = std::atan2(2.f*(qw*qy - qx*qz), 1.f - 2.f*(qy*qy + qz*qz));
    float cy = std::cos(yaw), sy = std::sin(yaw);

    const auto &robot_indices = world.GetRobotIndices();
    auto &npq = world.GetNarrowPhaseQuery();

    BPFilterStatic  bp_static;
    BPFilterDynamic bp_dynamic;
    BPFilterAll     bp_all;
    ObjFilterAll    obj_all;
    JPH::RayCastSettings ray_settings;

    for (int ri = 0; ri < (int)m_cfg->rays.size(); ++ri)
    {
        const RayDef &rd = m_cfg->rays[ri];

        float lx = rd.origin_offset[0];
        float ly = rd.origin_offset[1];
        float lz = rd.origin_offset[2];

        float ox = robot.pos[0] + lx * cy - lz * sy;
        float oy = m_cfg->origin_y + ly;
        float oz = robot.pos[2] + lx * sy + lz * cy;

        float world_yaw = yaw + rd.yaw_deg * (JPH::JPH_PI / 180.f);
        float pitch_rad =       rd.pitch_deg * (JPH::JPH_PI / 180.f);
        float horiz = std::cos(pitch_rad);
        float dx    = horiz * std::cos(world_yaw);
        float dy    = -std::sin(pitch_rad);
        float dz    = horiz * std::sin(world_yaw);

        JPH::RRayCast ray{
            JPH::RVec3(ox, oy, oz),
            JPH::Vec3(dx * rd.max_dist, dy * rd.max_dist, dz * rd.max_dist)
        };

        const JPH::BroadPhaseLayerFilter *bp_filter =
            rd.target == RayTarget::FIELD  ? (JPH::BroadPhaseLayerFilter*)&bp_static  :
            rd.target == RayTarget::PIECE  ? (JPH::BroadPhaseLayerFilter*)&bp_dynamic :
                                             (JPH::BroadPhaseLayerFilter*)&bp_all;

        JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
        npq.CastRay(ray, ray_settings, collector, *bp_filter, obj_all);

        float norm_dist = 1.0f;
        if (collector.HadHit())
        {
            JPH::BodyID hit_id = collector.mHit.mBodyID;
            bool skip = false;
            if (rd.target != RayTarget::FIELD) {
                for (int ridx : robot_indices)
                    if (world.GetBodyID(ridx) == hit_id) { skip = true; break; }
            }
            if (!skip)
                norm_dist = collector.mHit.mFraction;
        }

        m_hits[ri] = norm_dist;

        auto &rout = m_render[ri];
        rout.origin[0]=ox; rout.origin[1]=oy; rout.origin[2]=oz;
        rout.hit[0] = ox + dx * rd.max_dist * norm_dist;
        rout.hit[1] = oy + dy * rd.max_dist * norm_dist;
        rout.hit[2] = oz + dz * rd.max_dist * norm_dist;
        rout.did_hit = (norm_dist < 1.0f);
        rout.color[0]=rd.color[0]; rout.color[1]=rd.color[1];
        rout.color[2]=rd.color[2]; rout.color[3]=rd.color[3];
    }

    m_pub.Set(std::span<const float>(m_hits));
}