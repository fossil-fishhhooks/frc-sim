#include "physics/ForceApplicator.h"
#include "io/EasyLog.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>

#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// Drive-only physics model
// ─────────────────────────────────────────────────────────────────────────────
//
// Every motor in a BodyDef is a drive wheel. No arm / mechanism path.
//
// MOTOR TORQUE
//   T = T_stall * (V - ω/ω_free)   continuous, symmetric
//
// SHAFT SPEED FROM BODY KINEMATICS
//   ω_shaft = (world_dir · v_contact) / r_wheel * gear_ratio
//
// PER-WHEEL NORMAL FORCE
//   Inverse-distance weighting from COM (horizontal plane) over all
//   grounded wheel attachment points. Wheels not touching carry nothing.
//
// TRACTION LIMIT
//   |F_drive| ≤ μ_dynamic * N_wheel   (friction circle, drive axis)
//
// LATERAL FRICTION — EFFECTIVE MASS IMPULSE
//   Cancels contact-point lateral velocity each tick, capped by friction
//   circle remainder. Uses effective mass at the contact point so no
//   manual wheel-count division is needed:
//     m_eff = 1 / (1/m + (r × lat̂) · I⁻¹ · (r × lat̂))
//
// VERTICAL COMPONENT STRIP
//   cv_horiz = contact_vel − UP*(UP·contact_vel)
//   Prevents impact velocity from generating spurious lateral forces.
// ─────────────────────────────────────────────────────────────────────────────

ForceApplicator::ForceApplicator(SimWorld &world,
                                 const MotorRegistry &motors,
                                 ContactListener &contacts)
    : m_world(world), m_motors(motors), m_contacts(contacts)
{
}

void ForceApplicator::Apply(float dt)
{
    static bool logged = false;
    if (!logged)
    {
        LOG_INFO("ForceApplicator: drive-only mode, %d bodies", m_world.BodyCount());
        logged = true;
    }

    auto &bi = m_world.GetBodyInterface();
    static const JPH::Vec3 UP(0.0f, 1.0f, 0.0f);

    for (int body_idx = 0; body_idx < m_world.BodyCount(); ++body_idx)
    {
        const BodyDef *body_def = m_world.GetBodyDef(body_idx);
        if (!body_def || body_def->motors.empty() || body_def->mass == 0.0f)
            continue;

        JPH::BodyID jph_id = m_world.GetBodyID(body_idx);
        if (jph_id.IsInvalid())
            continue;

        JPH::RMat44 body_transform = bi.GetWorldTransform(jph_id);
        JPH::Vec3   body_com       = bi.GetCenterOfMassPosition(jph_id);
        JPH::Vec3   body_vel       = bi.GetLinearVelocity(jph_id);
        JPH::Vec3   body_ang_vel   = bi.GetAngularVelocity(jph_id);
        float       body_mass      = body_def->mass;
        float       inv_mass       = 1.0f / body_mass;
        JPH::Mat44  inv_inertia    = bi.GetInverseInertia(jph_id);

        int n_motors = std::min((int)body_def->motors.size(),
                                (int)SimWorld::MAX_MOTORS_PER_BODY);

        // ── Groundedness & per-wheel normal force ─────────────────────────
        float total_normal  = m_contacts.GetNormalForce(jph_id);
        bool  body_grounded = (total_normal >= MIN_NORMAL_FORCE);

        struct WheelData { int idx; JPH::Vec3 world_att; float normal; };
        WheelData wheel_data[SimWorld::MAX_MOTORS_PER_BODY];
        int n_wheels = 0;

        if (body_grounded)
        {
            for (int m = 0; m < n_motors; ++m)
            {
                const auto &md = body_def->motors[m];
                JPH::Vec3 watt = body_transform *
                                 JPH::Vec3(md.attachment[0],
                                           md.attachment[1],
                                           md.attachment[2]);
                wheel_data[n_wheels++] = { m, watt, 0.0f };
            }

            // Inverse-distance weighting from COM in the horizontal plane
            float weights[SimWorld::MAX_MOTORS_PER_BODY];
            float wsum = 0.0f;
            for (int i = 0; i < n_wheels; ++i)
            {
                JPH::Vec3 d = wheel_data[i].world_att - body_com;
                float hdist = std::sqrt(d.GetX()*d.GetX() + d.GetZ()*d.GetZ());
                weights[i] = 1.0f / (hdist + 0.01f);
                wsum += weights[i];
            }
            for (int i = 0; i < n_wheels; ++i)
                wheel_data[i].normal = total_normal * (weights[i] / wsum);
        }

        // ── Per-motor (all wheels) ────────────────────────────────────────
        for (int m = 0; m < n_motors; ++m)
        {
            const MotorAttachmentDef &md     = body_def->motors[m];
            const MotorModel         *profile = m_motors.Lookup(md.profile_name);
            if (!profile) continue;

            float voltage  = m_world.GetMotorVoltage(body_idx, m);
            float max_omega = profile->free_speed * md.gear_ratio;

            JPH::Vec3 world_dir = body_transform.Multiply3x3(
                                      JPH::Vec3(md.direction[0],
                                                md.direction[1],
                                                md.direction[2])).Normalized();
            JPH::Vec3 world_att = body_transform *
                                  JPH::Vec3(md.attachment[0],
                                            md.attachment[1],
                                            md.attachment[2]);

            // Contact point velocity and shaft speed from kinematics
            JPH::Vec3 contact_vel  = body_vel + body_ang_vel.Cross(world_att - body_com);
            float     ground_speed = world_dir.Dot(contact_vel);
            float     omega_shaft  = (ground_speed / md.wheel.radius) * md.gear_ratio;
            omega_shaft = std::clamp(omega_shaft, -max_omega, max_omega);

            // ── AIRBORNE ──────────────────────────────────────────────────
            if (!body_grounded)
            {
                m_world.SetMotorNormalForce  (body_idx, m, 0.0f);
                m_world.SetMotorTractiveForce(body_idx, m, 0.0f);
                m_world.SetMotorSlipping     (body_idx, m, false);
                m_world.SetMotorOmega        (body_idx, m, omega_shaft);
                continue;
            }

            // ── GROUNDED ─────────────────────────────────────────────────
            float normal_per_wheel = 0.0f;
            for (int i = 0; i < n_wheels; ++i)
                if (wheel_data[i].idx == m) { normal_per_wheel = wheel_data[i].normal; break; }

            // Drive force, clamped to traction limit
            float motor_torque  = profile->torque_at(omega_shaft, voltage);
            float force_mag     = (motor_torque * md.gear_ratio) / md.wheel.radius;
            float friction_cap  = md.wheel.cof_dynamic * normal_per_wheel;

            bool slipping = std::abs(force_mag) > friction_cap;
            if (slipping)
                force_mag = std::copysign(friction_cap, force_mag);

            bi.AddForce(jph_id, world_dir * force_mag, world_att);

            m_world.SetMotorNormalForce  (body_idx, m, normal_per_wheel);
            m_world.SetMotorTractiveForce(body_idx, m, force_mag);
            m_world.SetMotorSlipping     (body_idx, m, slipping);
            m_world.SetMotorOmega        (body_idx, m, omega_shaft);

            // ── Lateral friction (effective-mass impulse) ─────────────────
            JPH::Vec3 cv_horiz = contact_vel - UP * UP.Dot(contact_vel);
            JPH::Vec3 v_lat    = cv_horiz - world_dir * world_dir.Dot(cv_horiz);
            float     lat_speed = v_lat.Length();

            float lat_cap = std::sqrt(std::max(0.0f,
                                friction_cap * friction_cap - force_mag * force_mag));

            if (lat_speed > 1e-4f)
            {
                JPH::Vec3 lat_dir   = v_lat / lat_speed;
                JPH::Vec3 r         = world_att - body_com;
                JPH::Vec3 r_cross   = r.Cross(lat_dir);
                JPH::Vec3 i_r_cross = inv_inertia.Multiply3x3(r_cross);
                float     rot_term  = r_cross.Dot(i_r_cross);
                float     m_eff     = 1.0f / (inv_mass + rot_term);

                float f_lat = std::min(lat_cap, m_eff * lat_speed / dt);
                bi.AddForce(jph_id, -lat_dir * f_lat, world_att);
            }

            static int tick_count = 0;
            if (body_idx == m_world.RobotIndex() && ++tick_count % 499 == 0)
            {
                LOG_INFO("Motor %d: T=%.2fNm  F=%.1fN  N=%.1fN  "
                         "omega=%.1f rad/s  V=%.2f  slip=%d",
                         m, motor_torque, force_mag, normal_per_wheel,
                         omega_shaft, voltage, (int)slipping);
            }
        }
    }
}
