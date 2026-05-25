#include "io/NTClient.h"
#include "io/EasyLog.h"
#include "physics/MechanismSystem.h"

#include <networktables/NetworkTableInstance.h>
#include <networktables/FloatTopic.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/BooleanTopic.h>

#include <string>
#include <span>
#include <vector>

// ── pImpl ─────────────────────────────────────────────────────────────────────

struct NTClient::Impl
{
    nt::NetworkTableInstance inst;

    // ── Drivetrain ────────────────────────────────────────────────────────
    struct MotorTopics {
        nt::FloatSubscriber     voltage_sub;
        nt::FloatPublisher      omega_pub;
        nt::FloatArrayPublisher position_pub;
        nt::FloatArrayPublisher direction_pub;
    };
    std::vector<MotorTopics> motors;

    // ── Intake ────────────────────────────────────────────────────────────
    nt::IntegerPublisher intake_held_pub;
    nt::IntegerPublisher intake_capacity_pub;

    // ── Shooter ───────────────────────────────────────────────────────────
    nt::BooleanSubscriber     fire_sub;
    nt::FloatSubscriber       speed_sub;
    nt::FloatArraySubscriber  direction_sub;
};

// ── NTClient ──────────────────────────────────────────────────────────────────

NTClient::NTClient()  = default;
NTClient::~NTClient() { Shutdown(); }

void NTClient::Init(const std::string &host, int port,
                    SimWorld &world, int robot_motor_count,
                    MechanismSystem *mechanisms)
{
    m_world             = &world;
    m_mechanisms        = mechanisms;
    m_robot_motor_count = robot_motor_count;
    m_impl              = new Impl();

    auto &inst = m_impl->inst;
    inst = nt::NetworkTableInstance::GetDefault();
    inst.StartClient4("FRC Simulation 3D");
    inst.SetServer(host.c_str(), port);

    // ── Drivetrain topics ─────────────────────────────────────────────────
    m_impl->motors.resize(robot_motor_count);
    for (int i = 0; i < robot_motor_count; ++i)
    {
        std::string base = "/sim/motors/" + std::to_string(i);
        auto &t = m_impl->motors[i];
        t.voltage_sub   = inst.GetFloatTopic(base + "/voltage").Subscribe(0.0f);
        t.omega_pub     = inst.GetFloatTopic(base + "/omega").Publish();
        t.position_pub  = inst.GetFloatArrayTopic(base + "/position").Publish();
        t.direction_pub = inst.GetFloatArrayTopic(base + "/direction").Publish();
    }

    // ── Intake topics ─────────────────────────────────────────────────────
    if (mechanisms)
    {
        m_impl->intake_held_pub     = inst.GetIntegerTopic("/sim/intake/held").Publish();
        m_impl->intake_capacity_pub = inst.GetIntegerTopic("/sim/intake/capacity").Publish();
    }

    // ── Shooter topics ────────────────────────────────────────────────────
    if (mechanisms)
    {
        m_impl->fire_sub      = inst.GetBooleanTopic("/sim/shooter/fire")
                                    .Subscribe(false);
        m_impl->speed_sub     = inst.GetFloatTopic("/sim/shooter/speed")
                                    .Subscribe(10.0f);
        m_impl->direction_sub = inst.GetFloatArrayTopic("/sim/shooter/direction")
                                    .Subscribe({});
    }

    LOG_INFO("NTClient: connecting to %s:%d (%d motor slots, mechanisms=%s)",
             host.c_str(), port, robot_motor_count,
             mechanisms ? "yes" : "no");
}

void NTClient::Shutdown()
{
    if (!m_impl) return;
    m_impl->inst.StopClient();
    delete m_impl;
    m_impl = nullptr;
    LOG_INFO("NTClient: shutdown");
}

void NTClient::Tick(const WorldSnapshot &snapshot)
{
    if (!m_impl || !m_world) return;

    // ── Connection status ─────────────────────────────────────────────────
    m_connected.store(
        m_impl->inst.GetNetworkMode() != NT_NET_MODE_NONE &&
        !m_impl->inst.GetConnections().empty()
    );

    int robot_idx = m_world->RobotIndex();
    if (robot_idx < 0) return;

    // ── Read voltages → SimWorld ──────────────────────────────────────────
    bool got_update = false;
    for (int i = 0; i < m_robot_motor_count; ++i)
    {
        // GetAtomic() returns a timestamped value — use it to detect fresh data
        auto val = m_impl->motors[i].voltage_sub.GetAtomic();
        float v = std::clamp(val.value, -1.0f, 1.0f);
        m_world->SetMotorVoltage(robot_idx, i, v);
        if (val.time > 0)
            got_update = true;
    }
    if (got_update)
    {
        m_last_rx_ms.store(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    // ── Publish motor state from snapshot ────────────────────────────────
    if (snapshot.robot_index < 0 ||
        snapshot.robot_index >= (int)snapshot.bodies.size()) goto mechanisms;

    {
        const auto &robot = snapshot.bodies[snapshot.robot_index];
        for (int i = 0;
             i < (int)robot.motors.size() && i < m_robot_motor_count; ++i)
        {
            const MotorSnapshot &m = robot.motors[i];
            auto &t = m_impl->motors[i];
            t.omega_pub.Set(m.omega);
            const float pos[3] = { m.position[0],  m.position[1],  m.position[2]  };
            const float dir[3] = { m.direction[0], m.direction[1], m.direction[2] };
            t.position_pub .Set(std::span<const float>(pos, 3));
            t.direction_pub.Set(std::span<const float>(dir, 3));
        }
    }

mechanisms:
    if (!m_mechanisms) return;

    // ── Publish intake state ──────────────────────────────────────────────
    m_impl->intake_held_pub.Set(m_mechanisms->HeldCount());
    // capacity is constant; republish every tick (cheap, and NT deduplicates)
    m_impl->intake_capacity_pub.Set(snapshot.intake_max_capacity);

    // ── Read shooter commands ─────────────────────────────────────────────
    bool fire_now = m_impl->fire_sub.Get();
    if (fire_now && !m_last_fire_val)
    {
        // Rising edge → arm a shot
        float speed = m_impl->speed_sub.Get();

        float dx = 0.f, dy = 0.707f, dz = 0.707f;
        auto dir_arr = m_impl->direction_sub.Get();
        if (dir_arr.size() >= 3) { dx = dir_arr[0]; dy = dir_arr[1]; dz = dir_arr[2]; }

        m_mechanisms->ArmShot(speed, dx, dy, dz);
        LOG_INFO("NTClient: shot armed  speed=%.1f dir=(%.2f,%.2f,%.2f)",
                 speed, dx, dy, dz);
    }
    m_last_fire_val = fire_now;
}
