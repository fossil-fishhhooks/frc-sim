#include <Jolt/Jolt.h>

#include "io/Raycaster.h"
#include "core/SimWorld.h"
#include "io/EasyLog.h"

#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Body/BodyFilter.h>

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
    if (m_robot_slot >= (int)ri_vec.size()) { LOG_INFO("Raycaster: slot OOB"); return; }
    int robot_idx = ri_vec[m_robot_slot];
    if (robot_idx >= (int)snapshot.bodies.size()) { LOG_INFO("Raycaster: robot_idx OOB"); return; }

    const BodySnapshot &robot = snapshot.bodies[robot_idx];
    JPH::Quat robot_rot(robot.rot[0], robot.rot[1], robot.rot[2], robot.rot[3]);
    JPH::RVec3 robot_pos(robot.pos[0], robot.pos[1], robot.pos[2]);

    auto &npq = world.GetNarrowPhaseQuery();

    BPFilterStatic  bp_static;
    BPFilterDynamic bp_dynamic;
    BPFilterAll     bp_all;
    ObjFilterAll    obj_all;
    JPH::RayCastSettings ray_settings;

    const auto &robot_indices = world.GetRobotIndices();
    std::vector<JPH::BodyID> robot_body_ids;
    robot_body_ids.reserve(robot_indices.size());
    for (int ridx : robot_indices)
        robot_body_ids.push_back(world.GetBodyID(ridx));

    for (int ri = 0; ri < (int)m_cfg->rays.size(); ++ri)
    {
        const RayDef &rd = m_cfg->rays[ri];

        JPH::Vec3 local_origin(rd.origin_offset[0],
                               rd.origin_offset[1] + m_cfg->origin_y,
                               rd.origin_offset[2]);
        JPH::Vec3 world_origin_offset = robot_rot * local_origin;
        JPH::RVec3 ray_origin = robot_pos + world_origin_offset;

        float yaw_rad   = rd.yaw_deg   * (JPH::JPH_PI / 180.f);
        float pitch_rad = rd.pitch_deg * (JPH::JPH_PI / 180.f);
        float horiz = std::cos(pitch_rad);
        JPH::Vec3 local_dir(horiz * std::cos(yaw_rad),
                            -std::sin(pitch_rad),
                            horiz * std::sin(yaw_rad));
        JPH::Vec3 world_dir = robot_rot * local_dir;

        JPH::RRayCast ray{ray_origin, world_dir * rd.max_dist};

        const JPH::BroadPhaseLayerFilter *bp_filter =
            rd.target == RayTarget::FIELD  ? (JPH::BroadPhaseLayerFilter*)&bp_static  :
            rd.target == RayTarget::PIECE  ? (JPH::BroadPhaseLayerFilter*)&bp_dynamic :
                                             (JPH::BroadPhaseLayerFilter*)&bp_all;

        JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
        if (rd.target != RayTarget::FIELD && !robot_body_ids.empty()) {
            struct SkipRobotBodies : public JPH::BodyFilter {
                const std::vector<JPH::BodyID> &ids;
                SkipRobotBodies(const std::vector<JPH::BodyID> &ids) : ids(ids) {}
                bool ShouldCollide(const JPH::BodyID &id) const override {
                    for (auto &rid : ids)
                        if (rid == id) return false;
                    return true;
                }
            } skip_filter(robot_body_ids);
            npq.CastRay(ray, ray_settings, collector, *bp_filter, obj_all, skip_filter);
        } else {
            npq.CastRay(ray, ray_settings, collector, *bp_filter, obj_all);
        }

        float norm_dist = 1.0f;
        if (collector.HadHit())
            norm_dist = collector.mHit.mFraction;

        m_hits[ri] = norm_dist;

        auto &rout = m_render[ri];
        JPH::Vec3 hit_pt = ray_origin + world_dir * rd.max_dist * norm_dist;
        rout.origin[0] = ray_origin.GetX();
        rout.origin[1] = ray_origin.GetY();
        rout.origin[2] = ray_origin.GetZ();
        rout.hit[0] = hit_pt.GetX();
        rout.hit[1] = hit_pt.GetY();
        rout.hit[2] = hit_pt.GetZ();
        rout.did_hit = (norm_dist < 1.0f);
        rout.color[0]=rd.color[0]; rout.color[1]=rd.color[1];
        rout.color[2]=rd.color[2]; rout.color[3]=rd.color[3];
    }

    m_pub.Set(std::span<const float>(m_hits));
}