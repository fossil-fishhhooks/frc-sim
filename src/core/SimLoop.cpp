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
    auto prev = Clock::now();
    float accum = 0.0f;

    // Hz measurement
    int tick_count = 0;
    auto hz_timer = Clock::now();

    while (m_running)
    {
        auto now = Clock::now();
        float elapsed = static_cast<float>(Duration(now - prev).count());
        prev = now;

        // Apply speed multiplier  running at 2x doubles the physics time per wall second
        elapsed *= m_speed;

        // Clamp: never try to catch up more than 50ms worth of steps in one frame.
        // Prevents spiral-of-death if the machine hiccups.
        if (elapsed > 0.05f)
            elapsed = 0.05f;

        // Accumulate time across loop iterations.
        accum += elapsed;

        // Step physics in fixed increments
        while (accum >= m_fixed_dt)
        {
            if (m_forces)
                m_forces->Apply(m_fixed_dt);
            if (m_mechanisms)
                m_mechanisms->Tick(m_fixed_dt);
            m_world.Step(m_fixed_dt);
            accum -= m_fixed_dt;
            ++tick_count;
        }

        // Capture snapshot into back buffer, then flip atomically
        {
            std::lock_guard<std::mutex> lock(m_buf_mutex);
            int back = 1 - m_front.load(std::memory_order_relaxed);
            m_world.CaptureSnapshot(m_buf[back]);

            // Mechanism state lives in MechanismSystem, not SimWorld —
            // fill it here while we hold the buffer lock.
            if (m_mechanisms)
            {
                m_buf[back].intake_held = m_mechanisms->HeldCount();
                m_buf[back].intake_max_capacity = m_mechanisms->IntakeCapacity();
                m_buf[back].shooter_armed = m_mechanisms->IsFirePending();
            }

            m_front.store(back, std::memory_order_release);
        }

        // Update measured Hz once per second
        double hz_elapsed = Duration(now - hz_timer).count();
        if (hz_elapsed >= 1.0)
        {
            m_measured_hz.store(static_cast<float>(tick_count / hz_elapsed));
            tick_count = 0;
            hz_timer = now;
        }

        // Yield ~500µs so we don't burn a full core when physics is fast.
        // At 500Hz physics this loop runs ~2ms of real work; sleeping 500µs
        // keeps CPU usage reasonable while still being very responsive.
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}