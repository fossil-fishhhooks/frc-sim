#pragma once
#include "core/SimWorld.h"
#include "core/Snapshot.h"
#include "physics/ForceApplicator.h"
#include "physics/MechanismSystem.h"

#include <atomic>
#include <thread>
#include <mutex>

class SimLoop
{
public:
    // fixed_dt: physics timestep in seconds (e.g. 1/500 = 2ms)
    // speed:    sim speed multiplier (1.0 = realtime, 0.5 = half speed)
    explicit SimLoop(SimWorld &world,
                     ForceApplicator *forces = nullptr,
                     MechanismSystem *mechanisms = nullptr,
                     float fixed_dt = 1.0f / 500.0f,
                     float speed = 1.0f);
    ~SimLoop();

    void Start();
    void Stop();

    // Returns a copy of the latest snapshot. Safe from any thread.
    WorldSnapshot GetSnapshot() const;

    // Current measured physics tick rate (Hz), updated each second.
    float MeasuredHz() const { return m_measured_hz.load(); }
    float TargetHz() const { return 1.0f / m_fixed_dt; }

private:
    void Run();

    SimWorld &m_world;
    float m_fixed_dt;
    float m_speed;

    std::atomic<bool> m_running{false};
    std::thread m_thread;

    // Double buffer
    mutable std::mutex m_buf_mutex;
    WorldSnapshot m_buf[2];
    std::atomic<int> m_front{0};

    // Perf tracking
    std::atomic<float> m_measured_hz{0.0f};

    ForceApplicator *m_forces;
    MechanismSystem *m_mechanisms;
};