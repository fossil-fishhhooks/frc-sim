#pragma once
#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// DC Motor physics
// ─────────────────────────────────────────────────────────────────────────────
//
// The fundamental model for a brushed or brushless DC motor:
//
//   V_applied = V_back_emf + I * R_winding
//   V_back_emf = omega * (V_max / free_speed)        [back-EMF constant * omega]
//   I = (V_applied - V_back_emf) / R_winding
//   T = I * torque_constant = stall_torque * (V_applied - V_back_emf) / V_max
//
// Substituting:
//   T = stall_torque * (voltage_pct - omega / free_speed)
//
// This is the correct continuous form. Key properties:
//
//   voltage_pct =  1, omega =  0          → T = +stall_torque   (full forward torque)
//   voltage_pct =  1, omega =  free_speed → T = 0               (free-running)
//   voltage_pct =  1, omega >  free_speed → T < 0               (regenerative)
//   voltage_pct =  0, omega >  0          → T = -stall_torque * omega/free_speed
//                                           (regenerative braking — NOT zero!)
//   voltage_pct = -1, omega =  0          → T = -stall_torque   (full reverse torque)
//   voltage_pct = -1, omega = -free_speed → T = 0               (free-running reverse)
//
// The model is fully continuous through voltage = 0 and sign changes of omega.
// No special-casing needed. The clamp to ±stall_torque models magnetic
// saturation — in practice FRC motors hit current limits before that.
//
// Previous implementation had:
//   if (voltage_pct == 0) return 0;      ← WRONG: kills regenerative braking
//   eff_free = free_speed * v_abs;       ← divides by ~0 near zero voltage
//   T = stall_torque * V * (1 - omega_fwd / eff_free)  ← blows up, clamped to hide it
//
// The correct formula needs no guards, no sign decomposition, no abs():
//   T = stall_torque * (voltage_pct - omega / free_speed)
// ─────────────────────────────────────────────────────────────────────────────

struct MotorModel
{
    float stall_torque;  // Nm  at 0 rpm, full voltage
    float free_speed;    // rad/s at full voltage, no load
    float stall_current; // A   (for future current limiting)
    float free_current;  // A

    // Returns output torque in Nm.
    // omega       — motor shaft angular velocity (rad/s), signed.
    //               Positive = spinning in the +voltage direction.
    //               For a grounded wheel this should be omega_from_ground
    //               (derived from body kinematics), not a separately integrated state.
    // voltage_pct — commanded voltage fraction [-1.0, 1.0]
    float torque_at(float omega, float voltage_pct) const
    {
        // T = T_stall * (V - ω/ω_free)
        // Continuous, symmetric, correct at V=0 (regenerative braking).
        float torque = stall_torque * (voltage_pct - omega / free_speed);
        return std::clamp(torque, -stall_torque, stall_torque);
    }
};

// Source: WPILib motor characterization data.
namespace Motors
{
    constexpr MotorModel Kraken = {7.09f, 608.0f, 366.0f, 2.0f};
    constexpr MotorModel Falcon500 = {4.69f, 577.0f, 257.0f, 1.5f};
    constexpr MotorModel NEO = {2.60f, 594.0f, 105.0f, 1.8f};
    constexpr MotorModel NEO_845 = {16.00f, 59.4f, 105.0f, 1.8f}; // 10:1 reduction
    constexpr MotorModel NEO550 = {0.97f, 1774.0f, 100.0f, 1.4f};
    constexpr MotorModel NEO_Vortex = {3.60f, 565.0f, 211.0f, 3.6f};
    constexpr MotorModel CIM = {2.42f, 523.0f, 133.0f, 2.7f};
    constexpr MotorModel MiniCIM = {1.41f, 598.0f, 89.0f, 3.0f};
}