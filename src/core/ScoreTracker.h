#pragma once
#include "core/ScoringDef.h"
#include <atomic>
#include <unordered_set>
#include <vector>

class SimWorld;  // forward declare — no Jolt headers here

enum class MatchPhase { WAITING, COUNTDOWN, AUTO, TELEOP, ENDED };

class ScoreTracker {
public:
    void LoadZones(const std::vector<ScoringZoneDef> &zones);
    void StartMatch();
    void Tick(float dt, SimWorld &world);

    MatchPhase GetPhase()     const;
    float      GetMatchTime() const;
    float      GetCountdown() const;
    int        GetScore(int team) const;

    struct State {
        MatchPhase phase     = MatchPhase::WAITING;
        float      match_time = 0.f;
        float      countdown  = 3.f;
        float      remaining  = 0.f;
        int        score[2]   = {};
    };
    State GetState() const;

    const std::vector<ScoringZoneDef> &GetZones() const { return m_zones; }

    static constexpr float AUTO_DURATION   = 20.0f;
    static constexpr float TELEOP_DURATION = 115.0f;

private:
    std::vector<ScoringZoneDef>  m_zones;
    std::atomic<int>   m_score[2]   {};
    std::atomic<float> m_match_time {0};
    std::atomic<float> m_countdown  {3};
    std::atomic<int>   m_phase      {(int)MatchPhase::WAITING};

    std::unordered_set<uint32_t> m_scored_ids;

    std::unordered_set<uint32_t> m_in_zone_ids;
};