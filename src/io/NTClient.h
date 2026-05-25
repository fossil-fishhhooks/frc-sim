#pragma once
#include "core/SimWorld.h"
#include "core/Snapshot.h"
#include <string>
#include <atomic>
#include <chrono>

// Forward-declare ntcore types
namespace nt { class NetworkTableInstance; }

class MechanismSystem;   // optional — nullptr = no mechanisms


// NTClient — NetworkTables 4 IO.
//
// Drivetrain:
//   Subscribes:  /sim/motors/{i}/voltage      → SimWorld::SetMotorVoltage()
//   Publishes:   /sim/motors/{i}/omega        (rad/s)
//                /sim/motors/{i}/position     (local xyz, float[3])
//                /sim/motors/{i}/direction    (local xyz, float[3])
//
// Intake:
//   Publishes:   /sim/intake/held             (int)
//                /sim/intake/capacity         (int, constant after Init)
//
// Shooter:
//   Subscribes:  /sim/shooter/fire            (bool, rising edge = fire)
//                /sim/shooter/speed           (float, m/s)
//                /sim/shooter/direction       (float[3], robot-local unit vec)

class NTClient
{
public:
    NTClient();
    ~NTClient();

    // Connect to NT4 server.
    // mechanisms may be nullptr if the scene has no intake/shooter.
    void Init(const std::string &host, int port,
              SimWorld &world, int robot_motor_count,
              MechanismSystem *mechanisms = nullptr);

    void Shutdown();

    // Call once per render frame (~60 Hz).
    void Tick(const WorldSnapshot &snapshot, float dt);

    bool IsConnected() const { return m_connected.load(); }

    float Ping() const;

private:
    struct Impl;
    Impl *m_impl = nullptr;

    SimWorld         *m_world             = nullptr;
    MechanismSystem  *m_mechanisms        = nullptr;
    int               m_robot_motor_count = 0;
    std::atomic<bool> m_connected         { false };

    // Edge-detection for /sim/shooter/fire
    bool m_last_fire_val = false;
    float m_fire_cooldown = 0.0f; // seconds remaining until next shot allowed
    float m_fire_rate = 2.0f;     // copied from ShooterDef at Init
};
