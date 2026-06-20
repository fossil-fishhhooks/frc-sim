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
    sdtx_canvas((float)sapp_width(), (float)sapp_height());
    sdtx_origin(0, 0);
    sdtx_font(0);

    constexpr float PAD = 10.0f;
    constexpr float LINE = 18.0f;
    float y = PAD;
    char buf[256];

    auto text = [&](const char* s) { sdtx_puts(s); sdtx_crlf(); };
    auto text_col = [&](uint8_t r, uint8_t g, uint8_t b, const char* s) {
        sdtx_color3b(r, g, b); sdtx_puts(s); sdtx_crlf();
    };
    auto printf_col = [&](uint8_t r, uint8_t g, uint8_t b, const char* fmt, auto... args) {
        snprintf(buf, sizeof(buf), fmt, args...);
        sdtx_color3b(r, g, b); sdtx_puts(buf); sdtx_crlf();
    };

    // FPS
    sdtx_pos(PAD, y);
    double fps = sapp_frame_duration() > 0.0 ? 1.0 / sapp_frame_duration() : 0.0;
    printf_col(0, 255, 0, "FPS: %.0f", fps);
    y += LINE + 2;

    // Physics Hz
    sdtx_pos(PAD, y);
    snprintf(buf, sizeof(buf), "Physics: %.0f/%.0f Hz", sim_hz, target_hz);
    text_col(fabsf(target_hz - sim_hz) > 5.0f ? 255 : 192,
             fabsf(target_hz - sim_hz) > 5.0f ? 0 : 192,
             fabsf(target_hz - sim_hz) > 5.0f ? 0 : 192, buf);
    y += LINE;

    sdtx_pos(PAD, y);
    printf_col(192, 192, 192, "Physics Time: %.2f s", snapshot.sim_time);
    y += LINE;

    sdtx_pos(PAD, y);
    printf_col(192, 192, 192, "Wall Time: %.2f s",
               (logger::elapsed() - wall_time_offset_ms) / 1000.0f);
    y += LINE;

    float time_loss = ((logger::elapsed() - wall_time_offset_ms) / 1000.0f) - snapshot.sim_time;
    sdtx_pos(PAD, y);
    printf_col(time_loss > 2.9f ? 255 : 192, 0, 0, "Time Loss: %.2f s", time_loss);
    y += LINE;

    sdtx_pos(PAD, y);
    printf_col(192, 192, 192, "Bodies: %d  Robots: %d",
               (int)snapshot.bodies.size(), (int)snapshot.robot_indices.size());
    y += LINE + 4;

    // NT4
    sdtx_pos(PAD, y);
    text_col(nt_connected ? 0 : 255, nt_connected ? 255 : 0, 0,
             nt_connected ? "NT4  connected" : "NT4  disconnected");
    if (nt_connected) {
        int pc_r = 0, pc_g = 255, pc_b = 0;
        if (nt_ping_ms < 0) { pc_r=255; pc_g=165; pc_b=0; }
        else if (nt_ping_ms > 100) { pc_r=255; pc_g=0; pc_b=0; }
        else if (nt_ping_ms > 20) { pc_r=255; pc_g=255; pc_b=0; }
        if (nt_ping_ms < 0)
            snprintf(buf, sizeof(buf), "  (no data yet)");
        else
            snprintf(buf, sizeof(buf), "  %.1f ms", nt_ping_ms);
        sdtx_pos(PAD + 140, y);
        text_col(pc_r, pc_g, pc_b, buf);
    }
    y += LINE + 6;

    // Per-robot telemetry
    for (int ri = 0; ri < (int)snapshot.robot_indices.size(); ++ri) {
        int body_idx = snapshot.robot_indices[ri];
        if (body_idx < 0 || body_idx >= (int)snapshot.bodies.size())
            continue;
        const BodySnapshot& robot = snapshot.bodies[body_idx];

        sdtx_pos(PAD, y);
        printf_col(100, 180, 255, "--- ROBOT %d ---", ri);
        y += LINE;

        if (ri < (int)snapshot.robot_mech.size()) {
            const RobotMechSnapshot& mech = snapshot.robot_mech[ri];
            if (mech.intake_max_capacity > 0) {
                sdtx_pos(PAD, y);
                printf_col(255, 255, 255, "Hopper: %d / %d",
                           mech.intake_held, mech.intake_max_capacity);

                // Simple text bar
                int bar_chars = (int)(12.0f * mech.intake_held / mech.intake_max_capacity);
                if (bar_chars > 12) bar_chars = 12;
                char bar[16];
                for (int c = 0; c < bar_chars; ++c) bar[c] = '#';
                for (int c = bar_chars; c < 12; ++c) bar[c] = '.';
                bar[12] = '\0';
                sdtx_pos(PAD + 120, y);
                text_col(mech.intake_held == mech.intake_max_capacity ? 0 : 0,
                         mech.intake_held == mech.intake_max_capacity ? 0 : 255,
                         mech.intake_held == mech.intake_max_capacity ? 255 : 0, bar);
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
                sdtx_crlf();
                y += LINE;
            }
        }

        if (robot.motors.empty()) { y += 4; continue; }

        for (int i = 0; i < (int)robot.motors.size(); ++i) {
            const MotorSnapshot& m = robot.motors[i];
            sdtx_pos(PAD, y);
            printf_col(192, 192, 192, "[%d] %+6.1f rad/s", i, m.omega);

            constexpr float FREE_SPEED = 608.0f;
            float frac = std::clamp(m.omega / FREE_SPEED, -1.0f, 1.0f);
            int bar_chars = (int)(8.0f * fabsf(frac));
            if (bar_chars > 8) bar_chars = 8;
            char bar[12];
            for (int c = 0; c < bar_chars; ++c) bar[c] = '#';
            bar[bar_chars] = '\0';
            sdtx_pos(PAD + 150, y);
            text_col(m.slipping ? 255 : 135, m.slipping ? 0 : 206,
                     m.slipping ? 0 : 235, bar);
            if (m.slipping) {
                sdtx_pos(PAD + 240, y);
                text_col(255, 0, 0, "SLIP");
            }
            y += LINE;
        }
        y += 6;
    }

    // Scoreboard
    const auto& ss = snapshot.score_state;
    if (ss.phase != MatchPhase::WAITING) {
        int sw = sapp_width();
        constexpr int SW = 220;
        float bx = (float)(sw - SW - (int)PAD);
        float by = PAD;

        const char* phase_str = ss.phase == MatchPhase::COUNTDOWN ? "COUNTDOWN"
                              : ss.phase == MatchPhase::AUTO ? "AUTO"
                              : ss.phase == MatchPhase::TELEOP ? "TELEOP" : "ENDED";
        sdtx_pos(bx + SW/2 - strlen(phase_str)*4.0f, by + 6);
        text_col(192, 192, 192, phase_str);

        sdtx_pos(bx + 20, by + 22);
        printf_col(100, 149, 237, "%d", ss.score[0]);
        sdtx_pos(bx + SW - 50, by + 22);
        printf_col(220, 50, 47, "%d", ss.score[1]);
        sdtx_pos(bx + SW/2 - 5, by + 22);
        text_col(192, 192, 192, "-");

        if (ss.phase == MatchPhase::COUNTDOWN) {
            snprintf(buf, sizeof(buf), "%.0f", ceilf(ss.countdown));
            sdtx_pos(bx + SW/2 - strlen(buf)*4.0f, by + 42);
            text_col(255, 255, 0, buf);
        } else {
            int mins = (int)ss.remaining / 60;
            float secs = fmodf(ss.remaining, 60.0f);
            snprintf(buf, sizeof(buf), "%d:%05.2f", mins, secs);
            sdtx_pos(bx + SW/2 - strlen(buf)*4.0f, by + 42);
            text_col(255, 255, 255, buf);
        }
    }

    sdtx_draw();
}
