#pragma once
#include "core/SimWorld.h"
#include "core/MechanismDef.h"
#include <atomic>

class MechanismSystem
{
public:
    // robot_body_index: which body in SimWorld is THIS robot
    MechanismSystem(SimWorld    &world,
                    const IntakeDef  &intake,
                    const ShooterDef &shooter,
                    int robot_body_index);

    void Tick(float dt);

    void ArmShot(float speed, float dir_x, float dir_y, float dir_z);

    int  HeldCount()         const { return m_held.load(); }
    int  IntakeCapacity()    const { return m_intake.max_capacity; }
    bool IsFirePending()     const { return m_fire_pending.load(); }
    float GetShooterFireRate() const { return m_shooter.fire_rate; }
    void Reset();

private:
    SimWorld          &m_world;
    const IntakeDef   &m_intake;
    const ShooterDef  &m_shooter;
    int  m_robot_slot = 0;

    std::atomic<int>  m_held{0};
    std::atomic<bool> m_fire_pending{false};
    float             m_fire_speed{10.f};
    float             m_fire_dir[3]{0.f, 0.707f, 0.707f};

    void RunIntake();
    void RunShooter();
};