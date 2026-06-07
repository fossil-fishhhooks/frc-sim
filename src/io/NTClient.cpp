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

    std::atomic<int64_t> rtt2_us{-1}; // microseconds, -1 = no sync yet
    NT_Listener time_sync_listener{0};

    // ── Drivetrain ────────────────────────────────────────────────────────
    struct MotorTopics {
        nt::FloatSubscriber     voltage_sub;
        nt::FloatPublisher      omega_pub;
        nt::FloatArrayPublisher position_pub;
        nt::FloatArrayPublisher direction_pub;
        nt::FloatSubscriber     steer_angle_sub;
    };
    std::vector<MotorTopics> motors;

    // ── Intake ────────────────────────────────────────────────────────────
    nt::IntegerPublisher intake_held_pub;
    nt::IntegerPublisher intake_capacity_pub;

    // ── Shooter ───────────────────────────────────────────────────────────
    nt::BooleanSubscriber     fire_sub;
    nt::FloatSubscriber       speed_sub;
    nt::FloatArraySubscriber  direction_sub;

    // POSE
    nt::FloatPublisher pose_x_pub;
    nt::FloatPublisher pose_y_pub;
    nt::FloatPublisher pose_z_pub;
    nt::FloatPublisher pose_qx_pub;
    nt::FloatPublisher pose_qy_pub;
    nt::FloatPublisher pose_qz_pub;
    nt::FloatPublisher pose_qw_pub;
};

// ── NTClient ──────────────────────────────────────────────────────────────────

NTClient::NTClient()  = default;
NTClient::~NTClient() { Shutdown(); }

void NTClient::Init(const std::string &host, int port,
                    SimWorld &world, int robot_motor_count,
                    int robot_body_index,
                    MechanismSystem *mechanisms)
{
    m_robot_slot = (int)world.GetRobotIndices().size() - 1;
    m_world             = &world;
    m_mechanisms        = mechanisms;
    m_robot_motor_count = robot_motor_count;
    m_impl              = new Impl();

    auto &inst = m_impl->inst;
    inst = nt::NetworkTableInstance::Create();
    inst.StartClient4("FRC Simulation 3D");
    inst.SetServer(host.c_str(), port);

    // ── Drivetrain topics ─────────────────────────────────────────────────
    m_impl->motors.resize(robot_motor_count);
    for (int i = 0; i < robot_motor_count; ++i)
    {
        std::string base = "/sim/motors/" + std::to_string(i);
        auto &t = m_impl->motors[i];
        t.voltage_sub   = inst.GetFloatTopic(base + "/voltage").Subscribe(0.0f);
        t.steer_angle_sub = inst.GetFloatTopic(base + "/steer_angle").Subscribe(0.0f);
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
    m_fire_rate = mechanisms ? mechanisms->GetShooterFireRate() : 2.0f;

    LOG_INFO("NTClient: connecting to %s:%d (%d motor slots, mechanisms=%s)",
             host.c_str(), port, robot_motor_count,
             mechanisms ? "yes" : "no");

    // ── Pose topics ───────────────────────────────────────────────────────
    m_impl->pose_x_pub   = inst.GetFloatTopic("/sim/robot/x").Publish();
    m_impl->pose_y_pub   = inst.GetFloatTopic("/sim/robot/y").Publish();
    m_impl->pose_z_pub   = inst.GetFloatTopic("/sim/robot/z").Publish();
    m_impl->pose_qx_pub  = inst.GetFloatTopic("/sim/robot/qx").Publish();
    m_impl->pose_qy_pub  = inst.GetFloatTopic("/sim/robot/qy").Publish();
    m_impl->pose_qz_pub  = inst.GetFloatTopic("/sim/robot/qz").Publish();
    m_impl->pose_qw_pub  = inst.GetFloatTopic("/sim/robot/qw").Publish();

    m_impl->time_sync_listener = inst.AddTimeSyncListener(false,
                                                          [this](const nt::Event &e)
                                                          {
                                                              if (auto *ts = e.GetTimeSyncEventData())
                                                              {
                                                                  if (ts->valid)
                                                                      m_impl->rtt2_us.store(ts->rtt2);
                                                              }
                                                          });
    LOG_INFO("NTClient: set up latency measurement");
}

void NTClient::Shutdown()
{
    if (!m_impl) return;
    m_impl->inst.StopClient();
    if (m_impl->time_sync_listener)
        m_impl->inst.RemoveListener(m_impl->time_sync_listener);
    nt::NetworkTableInstance::Destroy(m_impl->inst); 
    delete m_impl;
    m_impl = nullptr;
    LOG_INFO("NTClient: shutdown");
}

float NTClient::Ping() const
{
    if (!m_impl)
        return -1.0f;
    int64_t rtt = m_impl->rtt2_us.load();
    if (rtt < 0)
        return -1.0f;
    return (float)(rtt / 1000.0); // us -> ms
}

void NTClient::Tick(const WorldSnapshot &snapshot, float dt)
{
    if (!m_impl || !m_world) return;

    // ── Connection status ─────────────────────────────────────────────────
    m_connected.store(
        m_impl->inst.GetNetworkMode() != NT_NET_MODE_NONE &&
        !m_impl->inst.GetConnections().empty()
    );

    const auto &ri_vec = m_world->GetRobotIndices();
    if (m_robot_slot < 0 || m_robot_slot >= (int)ri_vec.size()) return;
    int robot_idx = ri_vec[m_robot_slot];
    if (robot_idx < 0 || robot_idx >= m_world->BodyCount()) return;

    // ── Read voltages → SimWorld ──────────────────────────────────────────
    for (int i = 0; i < m_robot_motor_count; ++i)
    {
        float v = std::clamp(m_impl->motors[i].voltage_sub.Get(), -1.0f, 1.0f);
        m_world->SetMotorVoltage(robot_idx, i, v);

        float angle = m_impl->motors[i].steer_angle_sub.Get();
        m_world->SetMotorSteerAngle(robot_idx, i, angle);
    }

    // ── Publish motor state from snapshot ────────────────────────────────
    if (robot_idx < 0 ||
        robot_idx >= (int)snapshot.bodies.size()) return;

    {
        const auto &robot = snapshot.bodies[robot_idx];
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
    // ── Publish robot pose ────────────────────────────────────────────────
    if (robot_idx >= 0 &&
        robot_idx < (int)snapshot.bodies.size())
    {
        const BodySnapshot &robot = snapshot.bodies[robot_idx];

        m_impl->pose_x_pub.Set(robot.pos[0]);
        m_impl->pose_y_pub.Set(robot.pos[1]);
        m_impl->pose_z_pub.Set(robot.pos[2]);
        m_impl->pose_qx_pub.Set(robot.rot[0]);
        m_impl->pose_qy_pub.Set(robot.rot[1]);
        m_impl->pose_qz_pub.Set(robot.rot[2]);
        m_impl->pose_qw_pub.Set(robot.rot[3]);
    }

    if (!m_mechanisms) return;

    // ── Publish intake state ──────────────────────────────────────────────
    m_impl->intake_held_pub.Set(m_mechanisms->HeldCount());
    // capacity is constant; republish every tick (cheap, and NT deduplicates)
    m_impl->intake_capacity_pub.Set(m_mechanisms->IntakeCapacity());

    // ── Read shooter commands ─────────────────────────────────────────────
    bool fire_now = m_impl->fire_sub.Get();
    m_fire_cooldown -= dt;

    if (fire_now)
    {
        // First press fires immediately; held fires at fire_rate
        bool first_press = fire_now && !m_last_fire_val;
        bool cooldown_done = m_fire_rate <= 0.0f || m_fire_cooldown <= 0.0f;

        if (first_press || cooldown_done)
        {
            float speed = m_impl->speed_sub.Get();
            float dx = 0.f, dy = 0.707f, dz = 0.707f;
            auto dir_arr = m_impl->direction_sub.Get();
            if (dir_arr.size() >= 3)
            {
                dx = dir_arr[0];
                dy = dir_arr[1];
                dz = dir_arr[2];
            }
            m_mechanisms->ArmShot(speed, dx, dy, dz);

            m_fire_cooldown = m_fire_rate > 0.0f ? 1.0f / m_fire_rate : 0.0f;
            LOG_INFO("NTClient: shot armed  speed=%.1f dir=(%.2f,%.2f,%.2f)",
                     speed, dx, dy, dz);
        }
    }
    else
    {
        // Released — reset cooldown so next press fires immediately
        m_fire_cooldown = 0.0f;
    }
    m_last_fire_val = fire_now;
}