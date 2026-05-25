#include "core/SimLoop.h"
#include "io/EasyLog.h"

#include <chrono>

using Clock = std::chrono::steady_clock;
using Duration = std::chrono::duration<double>;

SimLoop::SimLoop(SimWorld &world, ForceApplicator *forces,
                 MechanismSystem *mechanisms,
                 float fixed_dt, float speed)
    : m_world(world), m_fixed_dt(fixed_dt), m_speed(speed),
      m_forces(forces), m_mechanisms(mechanisms)
{
}

SimLoop::~SimLoop() { Stop(); }

void SimLoop::Start()
{
    m_running = true;
    m_thread = std::thread(&SimLoop::Run, this);
    LOG_INFO("SimLoop: started (dt=%.4f s, speed=%.2fx)", m_fixed_dt, m_speed);
}

void SimLoop::Stop()
{
    if (!m_running)
        return;
    m_running = false;
    if (m_thread.joinable())
        m_thread.join();
    LOG_INFO("SimLoop: stopped");
}

WorldSnapshot SimLoop::GetSnapshot() const
{
    std::lock_guard<std::mutex> lock(m_buf_mutex);
    return m_buf[m_front.load(std::memory_order_acquire)];
}

void SimLoop::Run()
{
    using Dur = std::chrono::duration<double>;

    // Wall-time interval between ticks (accounts for speed multiplier)
    auto tick_interval = std::chrono::duration_cast<Clock::duration>(
        Dur(m_fixed_dt / m_speed));

    auto next_tick = Clock::now();
    int tick_count = 0;
    auto hz_timer = Clock::now();

    while (m_running)
    {
        std::this_thread::sleep_until(next_tick);
        next_tick += tick_interval;

        // If we're behind, reset — don't try to catch up
        auto now = Clock::now();
        if (now > next_tick)
            next_tick = now;

        // ── Physics step ──────────────────────────────────────────────
        if (m_forces)
            m_forces->Apply(m_fixed_dt);
        if (m_mechanisms)
            m_mechanisms->Tick(m_fixed_dt);
        m_world.Step(m_fixed_dt);
        ++tick_count;

        // ── Snapshot ──────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lock(m_buf_mutex);
            int back = 1 - m_front.load(std::memory_order_relaxed);
            m_world.CaptureSnapshot(m_buf[back]);
            if (m_mechanisms)
            {
                m_buf[back].intake_held = m_mechanisms->HeldCount();
                m_buf[back].intake_max_capacity = m_mechanisms->IntakeCapacity();
                m_buf[back].shooter_armed = m_mechanisms->IsFirePending();
            }
            m_front.store(back, std::memory_order_release);
        }

        // ── Hz measurement ────────────────────────────────────────────
        double hz_elapsed = Dur(Clock::now() - hz_timer).count();
        if (hz_elapsed >= 1.0)
        {
            m_measured_hz.store((float)(tick_count / hz_elapsed));
            tick_count = 0;
            hz_timer = Clock::now();
        }
    }
}