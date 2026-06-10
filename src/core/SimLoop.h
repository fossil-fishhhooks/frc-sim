#pragma once
#include "core/SimWorld.h"
#include "core/Snapshot.h"
#include "physics/ForceApplicator.h"
#include "physics/MechanismSystem.h"
#include "core/ScoreTracker.h"

#include <atomic>
#include <thread>
#include <mutex>
#include <vector>

class SimLoop
{
public:
    explicit SimLoop(SimWorld&, ForceApplicator*, std::vector<MechanismSystem*>, ScoreTracker*, float, float);
    ~SimLoop();

    void Start();
    void Stop();

    WorldSnapshot GetSnapshot() const;

    float MeasuredHz() const { return m_measured_hz.load(); }
    float TargetHz()   const { return 1.0f / m_fixed_dt; }

private:
    void Run();

    SimWorld        &m_world;
    float            m_fixed_dt;
    float            m_speed;

    std::atomic<bool> m_running{false};
    std::thread       m_thread;

    mutable std::mutex m_buf_mutex;
    WorldSnapshot      m_buf[2];
    std::atomic<int>   m_front{0};

    std::atomic<float> m_measured_hz{0.0f};

    ForceApplicator              *m_forces;
    std::vector<MechanismSystem*> m_mechanisms;   // nullptrs allowed

    ScoreTracker* m_score_tracker;
};