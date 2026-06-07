#pragma once
#include "core/SimWorld.h"
#include "core/Snapshot.h"
#include <string>
#include <atomic>
#include <chrono>

namespace nt { class NetworkTableInstance; }
class MechanismSystem;

// NTClient — one instance per robot, each connecting to that robot's NT4 server.
//
// Subscribes:  /sim/motors/{i}/voltage
//              /sim/motors/{i}/steer_angle
//              /sim/shooter/fire, /sim/shooter/speed, /sim/shooter/direction
//
// Publishes:   /sim/motors/{i}/omega, position, direction
//              /sim/intake/held, /sim/intake/capacity
//              /sim/robot/x, y, z, qx, qy, qz, qw, yaw

class NTClient
{
public:
    NTClient();
    ~NTClient();

    // robot_body_index: index of this robot's body in SimWorld::m_bodies
    void Init(const std::string &host, int port,
              SimWorld &world,
              int robot_motor_count,
              int robot_body_index,
              MechanismSystem *mechanisms = nullptr);

    void Shutdown();

    // Call once per render frame
    void Tick(const WorldSnapshot &snapshot, float dt);

    bool  IsConnected() const { return m_connected.load(); }
    float Ping()        const;

private:
    struct Impl;
    Impl *m_impl = nullptr;

    SimWorld        *m_world             = nullptr;
    MechanismSystem *m_mechanisms        = nullptr;
    int              m_robot_motor_count = 0;

    int m_robot_slot = 0;

    std::atomic<bool> m_connected{false};

    bool  m_last_fire_val = false;
    float m_fire_cooldown = 0.0f;
    float m_fire_rate     = 2.0f;
};