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




    // ── Scoreboard ────────────────────────────────────────────────────────
    const auto &ss = snapshot.score_state;
    if (ss.phase != MatchPhase::WAITING) {
        constexpr int SW  = 220;
        constexpr int SX  = 10;  // top-right anchored below
        int sw_x = GetScreenWidth() - SW - PAD;

        // measure height: header + scores + phase/time + active zones
        int active_zone_count = 0;
        for (const auto &z : snapshot.score_zones) {
            bool active = true;
            if (z.active_start >= 0.f && ss.match_time < z.active_start) active = false;
            if (z.active_end   >= 0.f && ss.match_time > z.active_end)   active = false;
            if (active) ++active_zone_count;
        }
        int box_h = 14 + 26 + 16 + 14 + (active_zone_count > 0 ? 14 + active_zone_count * 13 : 0) + 10;
        int bx = sw_x, by = PAD;

        // background
        DrawRectangle(bx, by, SW, box_h, {0, 0, 0, 160});
        DrawRectangleLines(bx, by, SW, box_h, {80, 80, 80, 200});

        int iy = by + 6;

        // phase label
        const char *phase_str = ss.phase == MatchPhase::COUNTDOWN ? "COUNTDOWN"
                              : ss.phase == MatchPhase::AUTO       ? "AUTO"
                              : ss.phase == MatchPhase::TELEOP     ? "TELEOP"
                              : "ENDED";
        DrawText(phase_str, bx + SW/2 - MeasureText(phase_str, 12)/2, iy, 12, LIGHTGRAY);
        iy += 16;

        // scores side by side
        snprintf(buf, sizeof(buf), "%d", ss.score[0]);
        DrawText(buf, bx + 20, iy, 26, {100, 149, 237, 255});  // blue
        snprintf(buf, sizeof(buf), "%d", ss.score[1]);
        DrawText(buf, bx + SW - 20 - MeasureText(buf, 26), iy, 26, {220, 50, 47, 255});  // red
        // divider dash
        DrawText("-", bx + SW/2 - MeasureText("-", 26)/2, iy, 26, LIGHTGRAY);
        iy += 30;

        // time
        if (ss.phase == MatchPhase::COUNTDOWN) {
            snprintf(buf, sizeof(buf), "%.0f", ceilf(ss.countdown));
            DrawText(buf, bx + SW/2 - MeasureText(buf, 16)/2, iy, 16, YELLOW);
        } else {
            int mins = (int)ss.remaining / 60;
            float secs = fmodf(ss.remaining, 60.f);
            snprintf(buf, sizeof(buf), "%d:%05.2f", mins, secs);
            DrawText(buf, bx + SW/2 - MeasureText(buf, 14)/2, iy, 14, WHITE);
        }
        iy += 18;

        // active zones
        if (active_zone_count > 0) {
            DrawLine(bx + 6, iy, bx + SW - 6, iy, {80, 80, 80, 160});
            iy += 6;
            DrawText("Active Zones", bx + 6, iy, 11, {160, 160, 160, 255});
            iy += 14;
            for (const auto &z : snapshot.score_zones) {
                bool active = true;
                if (z.active_start >= 0.f && ss.match_time < z.active_start) active = false;
                if (z.active_end   >= 0.f && ss.match_time > z.active_end)   active = false;
                if (!active) continue;
                Color zc = z.team == 0 ? Color{100, 149, 237, 255} : Color{220, 50, 47, 255};
                snprintf(buf, sizeof(buf), "%s (+%d)", z.id.c_str(), z.points);
                DrawText(buf, bx + 10, iy, 11, zc);
                iy += 13;
            }
        }
    }
}