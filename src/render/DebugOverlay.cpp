#include "render/DebugOverlay.h"
#include <raylib.h>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include "io/EasyLog.h"

static void DrawBar(int x, int y, int w, int h,
                    float fraction, Color fill, Color bg)
{
    DrawRectangle(x, y, w, h, bg);
    DrawRectangle(x, y, (int)(w * std::fabs(fraction)), h, fill);
    DrawRectangleLines(x, y, w, h, {80, 80, 80, 200});
}

void DrawDebugOverlay(const WorldSnapshot &snapshot,
                      bool  nt_connected,
                      float sim_hz, float target_hz,
                      float nt_ping_ms,
                      float wall_time_offset_ms)
{
    constexpr int PAD   = 10;
    constexpr int LINE  = 18;
    constexpr int SMALL = 14;

    int y = PAD;
    char buf[128];

    // ── FPS + sim Hz ──────────────────────────────────────────────────────
    DrawFPS(PAD, y);
    y += LINE + 2;

    snprintf(buf, sizeof(buf), "Physics:  %.0f/%.0f Hz", sim_hz, target_hz);
    Color speed_color = ((target_hz - sim_hz > 5.0f) || (sim_hz - target_hz > 2.0f))
                      ? RED : LIGHTGRAY;
    DrawText(buf, PAD, y, SMALL, speed_color);
    y += LINE;

    snprintf(buf, sizeof(buf), "Physics Time: %.2f s", snapshot.sim_time);
    DrawText(buf, PAD, y, SMALL, LIGHTGRAY);
    y += LINE;

    snprintf(buf, sizeof(buf), "Wall Time: %.2f s",
             (logger::elapsed() - wall_time_offset_ms) / 1000.0f);
    DrawText(buf, PAD, y, SMALL, LIGHTGRAY);
    y += LINE;

    float time_loss = ((logger::elapsed() - wall_time_offset_ms) / 1000.0f) - snapshot.sim_time;
    snprintf(buf, sizeof(buf), "Time Loss: %.2f s", time_loss);
    DrawText(buf, PAD, y, SMALL, time_loss > 2.9f ? RED : LIGHTGRAY);
    y += LINE;

    snprintf(buf, sizeof(buf), "Bodies: %d  Robots: %d",
             (int)snapshot.bodies.size(), (int)snapshot.robot_indices.size());
    DrawText(buf, PAD, y, SMALL, LIGHTGRAY);
    y += LINE + 4;

    // ── NT4 status ────────────────────────────────────────────────────────
    DrawCircle(PAD + 6, y + 6, 6, nt_connected ? GREEN : RED);
    DrawText(nt_connected ? "NT4  connected" : "NT4  disconnected",
             PAD + 18, y, SMALL, nt_connected ? GREEN : RED);
    if (nt_connected) {
        if (nt_ping_ms < 0)
            snprintf(buf, sizeof(buf), "  (no data yet)");
        else
            snprintf(buf, sizeof(buf), "  %.1f ms", nt_ping_ms);
        Color pc = nt_ping_ms < 0 ? ORANGE : nt_ping_ms > 100 ? RED
                 : nt_ping_ms > 20 ? YELLOW : GREEN;
        DrawText(buf, PAD + 140, y, SMALL, pc);
    }
    y += LINE + 6;

    // ── Per-robot telemetry ───────────────────────────────────────────────
    for (int ri = 0; ri < (int)snapshot.robot_indices.size(); ++ri)
    {
        int body_idx = snapshot.robot_indices[ri];
        if (body_idx < 0 || body_idx >= (int)snapshot.bodies.size())
            continue;

        const BodySnapshot &robot = snapshot.bodies[body_idx];

        // Robot header
        snprintf(buf, sizeof(buf), "--- ROBOT %d ---", ri);
        DrawText(buf, PAD, y, SMALL, {100, 180, 255, 255});
        y += LINE;

        // Mechanism state
        if (ri < (int)snapshot.robot_mech.size())
        {
            const RobotMechSnapshot &mech = snapshot.robot_mech[ri];
            if (mech.intake_max_capacity > 0)
            {
                snprintf(buf, sizeof(buf), "Hopper: %d / %d",
                         mech.intake_held, mech.intake_max_capacity);
                DrawText(buf, PAD, y, SMALL, WHITE);

                float frac = (float)mech.intake_held / mech.intake_max_capacity;
                Color ic   = mech.intake_held == mech.intake_max_capacity ? BLUE : GREEN;
                DrawBar(PAD + 120, y + 2, 60, LINE - 6, frac, ic, {40,40,40,200});
                y += LINE;

                DrawText("Shooter:", PAD, y, SMALL, WHITE);
                if (mech.shooter_armed)
                    DrawText("ACTIVE", PAD + 72, y, SMALL, ORANGE);
                else if (mech.intake_held > 0)
                    DrawText("READY",  PAD + 72, y, SMALL, GREEN);
                else
                    DrawText("EMPTY",  PAD + 72, y, SMALL, {80,80,80,200});
                y += LINE;
            }
        }

        if (robot.motors.empty()) {
            y += 4;
            continue;
        }

        // Motor bars
        for (int i = 0; i < (int)robot.motors.size(); ++i)
        {
            const MotorSnapshot &m = robot.motors[i];
            snprintf(buf, sizeof(buf), "[%d] %+6.1f rad/s", i, m.omega);
            DrawText(buf, PAD, y, SMALL, LIGHTGRAY);

            constexpr float FREE_SPEED = 608.0f;
            float frac = std::clamp(m.omega / FREE_SPEED, -1.0f, 1.0f);
            DrawBar(PAD + 150, y + 2, 80, LINE - 6,
                    frac, m.slipping ? RED : SKYBLUE, {40,40,40,200});
            if (m.slipping)
                DrawText("SLIP", PAD + 240, y, SMALL, RED);
            y += LINE;
        }

        y += 6; // gap between robots
    }




    //scoreboard
    // Centered scoreboard — only during/after match
    const auto &ss = snapshot.score_state;
    if (ss.phase != MatchPhase::WAITING) {
        // Big score numbers
        char t0[16], t1[16];
        snprintf(t0, sizeof(t0), "%d", ss.score[0]);
        snprintf(t1, sizeof(t1), "%d", ss.score[1]);
        int cx = GetScreenWidth() / 2;
        DrawText(t0, cx - 120, 20, 60, {100, 149, 237, 255});  // blue
        DrawText(t1, cx + 60,  20, 60, {220,  50,  47, 255});  // red

        // Match time or countdown
        if (ss.phase == MatchPhase::COUNTDOWN) {
            char cd[8]; snprintf(cd, sizeof(cd), "%.0f", ceilf(ss.countdown));
            int tw = MeasureText(cd, 80);
            DrawText(cd, cx - tw/2, 90, 80, YELLOW);
        } else {
            float remaining = (AUTO_DURATION + TELEOP_DURATION) - ss.match_time;
            char mt[16]; snprintf(mt, sizeof(mt), "%d:%05.2f",
                                (int)remaining/60, fmodf(remaining, 60.f));
            DrawText(mt, cx - MeasureText(mt, 24)/2, 85, 24, WHITE);
            // AUTO / TELEOP label
            const char *phase_str = (ss.phase == MatchPhase::AUTO) ? "AUTO" : "TELEOP";
            DrawText(phase_str, cx - MeasureText(phase_str,14)/2, 112, 14, LIGHTGRAY);
        }
    }
}