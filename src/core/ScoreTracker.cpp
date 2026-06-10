#include "core/ScoreTracker.h"
#include "core/SimWorld.h"
#include "io/EasyLog.h"

#include <Jolt/Jolt.h>
#include <Jolt/Geometry/AABox.h>
#include <Jolt/Physics/Body/BodyInterface.h>

static bool ZoneActive(const ScoringZoneDef &z, float t) {
    if (z.active_start >= 0.f && t < z.active_start) return false;
    if (z.active_end   >= 0.f && t > z.active_end)   return false;
    return true;
}

static bool BodyInZone(const ScoringZoneDef &z, const JPH::AABox &aabb) {
    for (int i = 0; i < 3; ++i) {
        if (aabb.mMax[i] < z.center[i] - z.half_extents[i]) return false;
        if (aabb.mMin[i] > z.center[i] + z.half_extents[i]) return false;
    }
    return true;
}

void ScoreTracker::LoadZones(const std::vector<ScoringZoneDef> &zones) {
    m_zones = zones;
}

void ScoreTracker::StartMatch() {
    m_score[0].store(0);
    m_score[1].store(0);
    m_match_time.store(0.f);
    m_countdown.store(3.f);
    m_scored_ids.clear();
    m_phase.store((int)MatchPhase::COUNTDOWN);
    LOG_INFO("ScoreTracker: match started (countdown)");
}

MatchPhase ScoreTracker::GetPhase()     const { return (MatchPhase)m_phase.load(); }
float      ScoreTracker::GetMatchTime() const { return m_match_time.load(); }
float      ScoreTracker::GetCountdown() const { return m_countdown.load(); }
int        ScoreTracker::GetScore(int t) const { return m_score[t & 1].load(); }

ScoreTracker::State ScoreTracker::GetState() const {
    float t = m_match_time.load();
    State s;
    s.phase      = GetPhase();
    s.match_time = t;
    s.countdown  = m_countdown.load();
    s.remaining  = std::max(0.f, (AUTO_DURATION + TELEOP_DURATION) - t);
    s.score[0]   = m_score[0].load();
    s.score[1]   = m_score[1].load();
    return s;
}

void ScoreTracker::Tick(float dt, SimWorld &world) {
    auto phase = (MatchPhase)m_phase.load();

    if (phase == MatchPhase::COUNTDOWN) {
        float cd = m_countdown.load() - dt;
        if (cd <= 0.f) {
            m_countdown.store(0.f);
            m_phase.store((int)MatchPhase::AUTO);
            LOG_INFO("ScoreTracker: AUTO start");
        } else {
            m_countdown.store(cd);
        }
        return;
    }

    if (phase != MatchPhase::AUTO && phase != MatchPhase::TELEOP) return;

    float t = m_match_time.load() + dt;
    m_match_time.store(t);

    if (phase == MatchPhase::AUTO && t >= AUTO_DURATION) {
        m_phase.store((int)MatchPhase::TELEOP);
        LOG_INFO("ScoreTracker: TELEOP start");
    }
    if (phase == MatchPhase::TELEOP && t >= AUTO_DURATION + TELEOP_DURATION) {
        m_phase.store((int)MatchPhase::ENDED);
        LOG_INFO("ScoreTracker: match ENDED  %d - %d",
                 m_score[0].load(), m_score[1].load());
        return;
    }

    // Score check
    const auto &robot_indices = world.GetRobotIndices();
    auto &bi = world.GetBodyInterface();

    for (int i = 0; i < world.BodyCount(); ++i) {
        // skip robots
        bool is_robot = false;
        for (int ri : robot_indices) if (i == ri) { is_robot = true; break; }
        if (is_robot) continue;

        JPH::BodyID bid = world.GetBodyID(i);
        if (bid.IsInvalid()) continue;
        if (bi.GetMotionType(bid) != JPH::EMotionType::Dynamic) continue;

        uint32_t uid = bid.GetIndexAndSequenceNumber();
        if (m_scored_ids.count(uid)) continue;

        JPH::AABox aabb = world.GetBodyAABB(bid);

        for (const auto &z : m_zones) {
            if (!ZoneActive(z, t)) continue;
            if (!BodyInZone(z, aabb)) continue;

            bool should_score = z.pass_through;
            if (!should_score) {
                JPH::Vec3 vel = bi.GetLinearVelocity(bid);
                should_score = vel.LengthSq() < 0.05f;
            }

            if (should_score) {
                m_score[z.team & 1].fetch_add(z.points);
                m_scored_ids.insert(uid);
                LOG_INFO("ScoreTracker: +%d team%d (zone '%s')  total=%d",
                         z.points, z.team, z.id.c_str(),
                         m_score[z.team & 1].load());
                break;
            }
        }
    }
}