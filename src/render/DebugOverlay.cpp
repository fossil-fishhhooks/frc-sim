#include "render/DebugOverlay.h"
#include "io/EasyLog.h"

#include <sokol_app.h>
#include <sokol_gfx.h>
#include <sokol_debugtext.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>

void DrawDebugOverlay(const WorldSnapshot& snapshot,
                      bool nt_connected,
                      float sim_hz, float target_hz,
                      float nt_ping_ms,
                      float wall_time_offset_ms) {
    float cw = (float)sapp_width() / 8.0f;
    float ch = (float)sapp_height() / 8.0f;
    sdtx_canvas(cw * 8.0f, ch * 8.0f);
    sdtx_font(0);

    constexpr float PAD = 1.0f;
    constexpr float LINE = 1.0f;
    float y = PAD;
    char buf[256];

    auto line = [&](uint8_t r, uint8_t g, uint8_t b, const char* fmt, auto... args) {
        sdtx_pos(PAD, y);
        snprintf(buf, sizeof(buf), fmt, args...);
        sdtx_color3b(r, g, b);
        sdtx_puts(buf);
        y += LINE;
    };

    // FPS
    double fps = sapp_frame_duration() > 0.0 ? 1.0 / sapp_frame_duration() : 0.0;
    line(0, 255, 0, "FPS: %.0f", fps);

    // Physics Hz
    uint8_t phz_r = 192, phz_g = 192, phz_b = 192;
    if (fabsf(target_hz - sim_hz) > 5.0f) { phz_r = 255; phz_g = 0; phz_b = 0; }
    else if (sim_hz > 0.0f) { phz_r = 0; phz_g = 255; phz_b = 0; }
    line(phz_r, phz_g, phz_b, "Physics: %.0f/%.0f Hz", sim_hz, target_hz);

    line(192, 192, 192, "Sim Time: %.2f s", snapshot.sim_time);

    float wall_sec = (logger::elapsed() - wall_time_offset_ms) / 1000.0f;
    line(192, 192, 192, "Wall Time: %.2f s", wall_sec);

    float time_loss = wall_sec - snapshot.sim_time;
    line(time_loss > 2.9f ? 255 : 192, 192, 192, "Time Loss: %.2f s", time_loss);

    line(192, 192, 192, "Bodies: %d  Robots: %d",
         (int)snapshot.bodies.size(), (int)snapshot.robot_indices.size());

    y += 0.5f;

    // NT4
    line(nt_connected ? 0 : 255, nt_connected ? 255 : 0, 0,
         nt_connected ? "NT4 connected" : "NT4 disconnected");
    if (nt_connected && nt_ping_ms >= 0) {
        uint8_t pr=0, pg=255, pb=0;
        if (nt_ping_ms > 100) { pr=255; pg=0; pb=0; }
        else if (nt_ping_ms > 20) { pr=255; pg=255; pb=0; }
        sdtx_pos(PAD + 15.0f, y - LINE);
        sdtx_color3b(pr, pg, pb);
        snprintf(buf, sizeof(buf), "%.0f ms", nt_ping_ms);
        sdtx_puts(buf);
    }

    y += 0.75f;

    // Per-robot telemetry
    for (int ri = 0; ri < (int)snapshot.robot_indices.size(); ++ri) {
        int body_idx = snapshot.robot_indices[ri];
        if (body_idx < 0 || body_idx >= (int)snapshot.bodies.size())
            continue;
        const BodySnapshot& robot = snapshot.bodies[body_idx];

        sdtx_pos(PAD, y);
        sdtx_color3b(100, 180, 255);
        snprintf(buf, sizeof(buf), "--- ROBOT %d ---", ri);
        sdtx_puts(buf);
        y += LINE;

        if (ri < (int)snapshot.robot_mech.size()) {
            const RobotMechSnapshot& mech = snapshot.robot_mech[ri];
            if (mech.intake_max_capacity > 0) {
                int pct = (int)(100.0f * mech.intake_held / mech.intake_max_capacity);
                sdtx_pos(PAD, y);
                sdtx_color3b(255, 255, 255);
                snprintf(buf, sizeof(buf), "Hopper: %d/%d (%d%%)",
                         mech.intake_held, mech.intake_max_capacity, pct);
                sdtx_puts(buf);
                y += LINE;

                sdtx_pos(PAD, y);
                sdtx_color3b(255, 255, 255);
                sdtx_puts("Shooter: ");
                if (mech.shooter_armed) {
                    sdtx_color3b(255, 165, 0); sdtx_puts("ACTIVE");
                } else if (mech.intake_held > 0) {
                    sdtx_color3b(0, 255, 0); sdtx_puts("READY");
                } else {
                    sdtx_color3b(80, 80, 80); sdtx_puts("EMPTY");
                }
                y += LINE;
            }
        }

        if (robot.motors.empty()) { y += 0.5f; continue; }

        sdtx_pos(PAD, y);
        sdtx_color3b(80, 80, 80);
        sdtx_puts("--- Motors ---");
        y += LINE;

        for (int i = 0; i < (int)robot.motors.size(); ++i) {
            const MotorSnapshot& m = robot.motors[i];

            if (m.slipping)
                sdtx_color3b(255, 0, 0);
            else
                sdtx_color3b(192, 192, 192);
            constexpr float FREE_SPEED = 608.0f;
            float frac = std::clamp(m.omega / FREE_SPEED, -1.0f, 1.0f);
            snprintf(buf, sizeof(buf), "(%d) %+5.0f rad/s %+4d%%%s", i, m.omega,
                     (int)(frac * 100.0f), m.slipping ? " SLIP" : "");
            sdtx_pos(PAD, y);
            sdtx_puts(buf);
            y += LINE;
        }
        y += 0.5f;
    }

    // Scoreboard (right side)
    const auto& ss = snapshot.score_state;
    if (ss.phase != MatchPhase::WAITING) {
        constexpr float SW = 24.0f;
        float bx = cw - SW - PAD;
        float by = PAD;

        const char* phase_str = ss.phase == MatchPhase::COUNTDOWN ? "COUNTDOWN"
                              : ss.phase == MatchPhase::AUTO ? "AUTO"
                              : ss.phase == MatchPhase::TELEOP ? "TELEOP" : "ENDED";
        float sx = bx + SW * 0.5f - strlen(phase_str) * 0.5f;
        sdtx_pos(sx, by);
        sdtx_color3b(192, 192, 192);
        sdtx_puts(phase_str);

        // Score row
        sdtx_pos(bx + 3.0f, by + 1.5f);
        sdtx_color3b(100, 149, 237);
        snprintf(buf, sizeof(buf), "%d", ss.score[0]);
        sdtx_puts(buf);

        sdtx_pos(bx + SW * 0.5f - 0.5f, by + 1.5f);
        sdtx_color3b(192, 192, 192);
        sdtx_puts("-");

        sdtx_pos(bx + SW - 4.0f, by + 1.5f);
        sdtx_color3b(220, 50, 47);
        snprintf(buf, sizeof(buf), "%d", ss.score[1]);
        sdtx_puts(buf);

        // Timer row
        if (ss.phase == MatchPhase::COUNTDOWN) {
            snprintf(buf, sizeof(buf), "%.0f", ceilf(ss.countdown));
            sdtx_color3b(255, 255, 0);
        } else {
            int mins = (int)ss.remaining / 60;
            float secs = fmodf(ss.remaining, 60.0f);
            snprintf(buf, sizeof(buf), "%d:%05.2f", mins, secs);
            sdtx_color3b(255, 255, 255);
        }
        float tx = bx + SW * 0.5f - strlen(buf) * 0.5f;
        sdtx_pos(tx, by + 3.0f);
        sdtx_puts(buf);
    }

    sdtx_draw();
}
