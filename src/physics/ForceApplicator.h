#pragma once
#include "core/SimWorld.h"
#include "core/MotorRegistry.h"
#include "physics/ContactListener.h"

#include <Jolt/Math/Vec3.h>
#include <unordered_map>
#include <cstdint>

// ForceApplicator: computes and applies drive-motor forces each physics tick.
//
// All motors in a BodyDef are treated as drive wheels. There is no
// "non-wheel" / arm / mechanism path — this sim is a pure drivebase sim.
// Mechanism behaviour (intake / shooter) is handled by MechanismSystem.

class ForceApplicator
{
public:
    ForceApplicator(SimWorld &world,
                    const MotorRegistry &motors,
                    ContactListener &contacts);

    // Call each tick before SimWorld::Step(dt).
    void Apply(float dt);

private:
    SimWorld            &m_world;
    const MotorRegistry &m_motors;
    ContactListener     &m_contacts;

    // Minimum normal force (N) to consider the body grounded.
    static constexpr float MIN_NORMAL_FORCE = 0.5f;

    // Previous-tick linear velocity per body (keyed by BodyID index+seq).
    // Used to estimate horizontal acceleration for weight-transfer calculation.
    // One tick behind — negligible at 500 Hz.
    std::unordered_map<uint32_t, JPH::Vec3> m_prev_vel;
};
