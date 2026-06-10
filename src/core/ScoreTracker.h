#pragma once
#include "ScoringDef.h"
#include <atomic>
#include <unordered_set>
#include <vector>
#include <Jolt/Geometry/AABox.h>
enum class MatchPhase { WAITING, COUNTDOWN, AUTO, TELEOP, ENDED };

class ScoreTracker {
public:
    void LoadZones(const std::vector<ScoringZoneDef> &zones);
    void StartMatch();   // called externally (NT trigger or keypress)
    void Tick(float dt, SimWorld &world);

    MatchPhase GetPhase()      const;
    float      GetMatchTime()  const;  // seconds elapsed since AUTO start
    float      GetCountdown()  const;  // 3..0 during COUNTDOWN
    int        GetScore(int team) const;

    // For snapshot
    struct State {
        MatchPhase phase;
        float      match_time;
        float      countdown;
        int        score[2];
    };
    State GetState() const;  // atomic read

private:
    std::vector<ScoringZoneDef> m_zones;
    std::atomic<int>   m_score[2]  {};
    std::atomic<float> m_match_time{0};
    std::atomic<float> m_countdown {3};
    std::atomic<int>   m_phase     {(int)MatchPhase::WAITING};

    std::unordered_set<uint32_t> m_scored_ids;  // BodyID indices already scored

    static constexpr float AUTO_DURATION    = 15.0f;
    static constexpr float TELEOP_DURATION  = 135.0f;
    static constexpr float COUNTDOWN_SECS   = 3.0f;

    bool ZoneActive(const ScoringZoneDef &z, float match_time) const;
    bool BodyInZone(const ScoringZoneDef &z, JPH::AABox aabb) const;
};