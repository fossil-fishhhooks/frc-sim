#include "physics/ForceApplicator.h"
#include "io/EasyLog.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>

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
// PER-WHEEL NORMAL FORCE — MOMENT-BALANCE WITH WEIGHT TRANSFER
//   Total normal force W comes from ContactListener (EstimateCollisionResponse).
//   It is distributed across n wheels via:
//
//     N_i = W/n + α·Δx_i + β·Δz_i
//
//   where (Δx_i, Δz_i) is wheel i's offset from COM in the horizontal world
//   plane, and (α, β) satisfy the pitch/roll moment equations:
//
//     α·Σ(Δx²) + β·Σ(Δx·Δz) = −m·a_x·h_cg − (W/n)·Σ(Δx)   [roll  moment]
//     α·Σ(Δx·Δz) + β·Σ(Δz²) = −m·a_z·h_cg − (W/n)·Σ(Δz)   [pitch moment]
//
//   a_x, a_z are the body's horizontal world-frame acceleration (1-tick
//   estimate from Δv/dt).  h_cg is the CG height above the wheel contact
//   plane.  Any N_i < 0 is clamped to 0 and the remainder is renormalized
//   to preserve the total W.  This handles symmetric and asymmetric layouts
//   and naturally captures weight transfer under acceleration/deceleration/turns.
//
// TRACTION LIMIT
//   |F_drive| ≤ μ_dynamic * N_wheel   (friction circle, drive axis)
//
// LATERAL FRICTION — EFFECTIVE MASS IMPULSE
//   Cancels contact-point lateral velocity each tick, capped by the friction
//   circle remainder sqrt(μ²N² − F_drive²).  Uses effective mass at the
//   contact point so no manual wheel-count division is needed:
//     m_eff = 1 / (1/m + (r × lat̂) · I⁻¹ · (r × lat̂))
//
// VERTICAL COMPONENT STRIP
//   cv_horiz = contact_vel − UP*(UP·contact_vel)
//   Prevents impact velocity from generating spurious lateral forces.
//
// NOTE — LINEAR/ANGULAR DAMPING
//   SimWorld applies flat 0.15/0.20 linear/angular damping on all dynamic
//   bodies.  This is a numerical stabilizer, not modeled aerodynamic drag.
//   Keep this in mind when comparing sim coast-down to real robot data.
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

        JPH::RMat44 body_transform;
        JPH::Vec3   body_com;
        JPH::Vec3   body_vel;
        JPH::Vec3   body_ang_vel;
        JPH::Mat44  inv_inertia;

        {
            JPH::BodyLockRead lock(m_world.GetBodyLockInterface(), jph_id);
            if (!lock.Succeeded()) continue;
            const JPH::Body &body = lock.GetBody();

            body_transform = body.GetWorldTransform();
            body_com       = body.GetCenterOfMassPosition();
            body_vel       = body.GetLinearVelocity();
            body_ang_vel   = body.GetAngularVelocity();
            inv_inertia    = body.GetInverseInertia();
        }

        float body_mass = body_def->mass;
        float inv_mass  = 1.0f / body_mass;

        int n_motors = std::min((int)body_def->motors.size(),
                                (int)SimWorld::MAX_MOTORS_PER_BODY);

        // ── Groundedness & per-wheel normal force ─────────────────────────
        float total_normal  = m_contacts.GetNormalForce(jph_id);
        bool  body_grounded = (total_normal >= MIN_NORMAL_FORCE);

        // Always track prev_vel regardless of groundedness, so the acceleration
        // estimate on landing uses the true previous tick — not stale pre-flight
        // data that would produce a huge spurious spike and wreck weight transfer.
        JPH::Vec3 prev_body_vel = body_vel; // default: zero accel if no history
        {
            uint32_t bkey = jph_id.GetIndexAndSequenceNumber();
            auto prev_it = m_prev_vel.find(bkey);
            if (prev_it != m_prev_vel.end())
                prev_body_vel = prev_it->second;
            m_prev_vel[bkey] = body_vel; // update for next tick
        }

        struct WheelData { int idx; JPH::Vec3 world_att; float normal; float dx; float dz; };
        WheelData wheel_data[SimWorld::MAX_MOTORS_PER_BODY];
        int n_wheels = 0;

        if (body_grounded)
        {
            // ── Collect wheel world positions ──────────────────────────────
            for (int m = 0; m < n_motors; ++m)
            {
                const auto &md = body_def->motors[m];
                JPH::Vec3 watt = body_transform *
                                 JPH::Vec3(md.attachment[0],
                                           md.attachment[1],
                                           md.attachment[2]);
                wheel_data[n_wheels++] = { m, watt, 0.0f, 0.0f, 0.0f };
            }

            // ── Moment-balance load distribution with weight transfer ───────
            // Distribute total_normal across wheels as
            //   N_i = W/n + α·Δx_i + β·Δz_i
            // where (α, β) satisfy the roll/pitch moment balance equations
            // including dynamic weight transfer from horizontal acceleration.

            // 1. CG height above the wheel contact plane.
            float att_y_sum = 0.0f;
            for (int i = 0; i < n_wheels; ++i)
                att_y_sum += wheel_data[i].world_att.GetY();
            float avg_att_y = att_y_sum / (float)n_wheels;
            float h_cg = std::max(0.05f, body_com.GetY() - avg_att_y);

            // 2. Horizontal acceleration estimate: Δv/dt, one tick behind.
            // prev_body_vel was captured before this grounded block (see above),
            // and updated every tick regardless of groundedness — so it always
            // reflects the true previous-tick velocity, even after airborne phases.
            float a_x = (body_vel.GetX() - prev_body_vel.GetX()) / dt;
            float a_z = (body_vel.GetZ() - prev_body_vel.GetZ()) / dt;

            // 3. Wheel offsets from COM (horizontal) and inertia matrix terms.
            float sum_dx2  = 0.0f, sum_dz2  = 0.0f;
            float sum_dxdz = 0.0f;
            float sum_dx   = 0.0f, sum_dz   = 0.0f;
            for (int i = 0; i < n_wheels; ++i)
            {
                float dx = wheel_data[i].world_att.GetX() - body_com.GetX();
                float dz = wheel_data[i].world_att.GetZ() - body_com.GetZ();
                wheel_data[i].dx = dx;
                wheel_data[i].dz = dz;
                sum_dx2  += dx * dx;
                sum_dz2  += dz * dz;
                sum_dxdz += dx * dz;
                sum_dx   += dx;
                sum_dz   += dz;
            }

            float W    = total_normal;
            float base = W / (float)n_wheels;

            // 4. Moment right-hand sides (dynamic weight transfer + static COM offset).
            //    Roll  moment (about world z): lateral  accel shifts weight left/right.
            //    Pitch moment (about world x): long.    accel shifts weight front/rear.
            float M_roll  = -body_mass * a_x * h_cg - base * sum_dx;
            float M_pitch = -body_mass * a_z * h_cg - base * sum_dz;

            // 5. Solve 2×2:  [sum_dx2   sum_dxdz] [α]   [M_roll ]
            //                [sum_dxdz  sum_dz2 ] [β] = [M_pitch]
            float det   = sum_dx2 * sum_dz2 - sum_dxdz * sum_dxdz;
            float alpha = 0.0f, beta = 0.0f;
            if (std::abs(det) > 1e-6f)
            {
                alpha = (M_roll  * sum_dz2  - M_pitch * sum_dxdz) / det;
                beta  = (M_pitch * sum_dx2  - M_roll  * sum_dxdz) / det;
            }

            // 6. Apply, clamp negatives, renormalize to preserve total W.
            float n_sum = 0.0f;
            for (int i = 0; i < n_wheels; ++i)
            {
                float N = base + alpha * wheel_data[i].dx + beta * wheel_data[i].dz;
                wheel_data[i].normal = std::max(0.0f, N);
                n_sum += wheel_data[i].normal;
            }
            if (n_sum > 1e-6f)
            {
                float scale = W / n_sum;
                for (int i = 0; i < n_wheels; ++i)
                    wheel_data[i].normal *= scale;
            }
        }

        // ── Per-motor (all wheels) ────────────────────────────────────────
        for (int m = 0; m < n_motors; ++m)
        {
            const MotorAttachmentDef &md     = body_def->motors[m];
            const MotorModel         *profile = m_motors.Lookup(md.profile_name);
            if (!profile) continue;

            float voltage  = m_world.GetMotorVoltage(body_idx, m);
            float max_omega = profile->free_speed * md.gear_ratio;

             JPH::Vec3 local_dir(md.direction[0], md.direction[1], md.direction[2]);
            if (md.is_steerable)
            {
                float sa = m_world.GetMotorSteerAngle(body_idx, m);
                float cs = std::cos(sa), sn = std::sin(sa);
                // Rotate around local +Y: x' = x*cos - z*sin, z' = x*sin + z*cos
                float rx = local_dir.GetX() * cs - local_dir.GetZ() * sn;
                float rz = local_dir.GetX() * sn + local_dir.GetZ() * cs;
                local_dir = JPH::Vec3(rx, local_dir.GetY(), rz);
            }

            JPH::Vec3 world_dir = body_transform.Multiply3x3(local_dir).Normalized();
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
            float cap_static  = md.wheel.cof_static  * normal_per_wheel;
            float cap_dynamic = md.wheel.cof_dynamic * normal_per_wheel;
            bool  slipping    = std::abs(force_mag)  > cap_static;
            float friction_cap = slipping ? cap_dynamic : cap_static;
            if (slipping)
                force_mag = std::copysign(cap_dynamic, force_mag);

            bi.AddImpulse(jph_id, world_dir* force_mag* dt, world_att);

            m_world.SetMotorNormalForce  (body_idx, m, normal_per_wheel);
            m_world.SetMotorTractiveForce(body_idx, m, force_mag);
            m_world.SetMotorSlipping     (body_idx, m, slipping);
            m_world.SetMotorOmega        (body_idx, m, omega_shaft);

            // ── Lateral friction (effective-mass impulse) ─────────────────
            JPH::Vec3 cv_horiz = contact_vel - UP * UP.Dot(contact_vel);
            JPH::Vec3 v_lat    = cv_horiz - world_dir * world_dir.Dot(cv_horiz);
            float     lat_speed = v_lat.Length();

            // Remaining lateral budget from the friction circle.
            // At full longitudinal slip (|F_drive| == friction_cap) this is
            // exactly zero — the tire is saturated and has no lateral reserve.
            float lat_cap = std::sqrt(std::max(0.0f,
                                friction_cap * friction_cap - force_mag * force_mag));

            if (lat_speed > 1e-4f)
            {
                JPH::Vec3 lat_dir = v_lat / lat_speed;
                JPH::Vec3 r = world_att - body_com;
                JPH::Vec3 r_cross = r.Cross(lat_dir);
                JPH::Vec3 i_r_cross = inv_inertia.Multiply3x3(r_cross);
                float     rot_term = r_cross.Dot(i_r_cross);
                float     m_eff = 1.0f / (inv_mass + rot_term);

                // Impulse to fully cancel lateral velocity this tick, capped by friction.
                // lat_cap is a force (N); convert to max impulse over this tick via *dt.
                // m_eff * lat_speed is the impulse needed to zero lateral velocity.
                // Take the smaller: don't apply more than friction allows, but also
                // don't apply more than needed.
                float j_lat = std::min(lat_cap * dt, m_eff * lat_speed);
                bi.AddImpulse(jph_id, -lat_dir * j_lat, world_att);
            }

            static int tick_count = 0;
            bool is_any_robot = false;
            for (int ri : m_world.GetRobotIndices()) if (body_idx == ri) { is_any_robot = true; break; }
            if (is_any_robot && ++tick_count % 499 == 0)
            {
                LOG_INFO("Body[%d] Motor %d: T=%.2fNm  F=%.1fN  N=%.1fN  "
                         "omega=%.1f rad/s  V=%.2f  slip=%d",
                         body_idx, m, motor_torque, force_mag, normal_per_wheel,
                         omega_shaft, voltage, (int)slipping);
            }
        }
    }
}
