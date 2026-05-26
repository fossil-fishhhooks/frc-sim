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
                      bool nt_connected,
                      float sim_hz, float target_hz, float nt_staleness_ms, float wall_time_offset_ms)
{
    constexpr int PAD = 10;
    constexpr int LINE = 18;
    constexpr int SMALL = 14;

    int y = PAD;
    char buf[128];

    // ── FPS + sim Hz ──────────────────────────────────────────────────────
    DrawFPS(PAD, y);
    y += LINE + 2;

    snprintf(buf, sizeof(buf), "Physics:  %.0f/%.0f Hz", sim_hz, target_hz);
    Color speed_color = ((target_hz - sim_hz > 5.0f) || (sim_hz - target_hz > 2.0f))
                            ? RED
                            : LIGHTGRAY;
    DrawText(buf, PAD, y, SMALL, speed_color);
    y += LINE;

    snprintf(buf, sizeof(buf), "Physics Time: %.2f s", snapshot.sim_time);
    DrawText(buf, PAD, y, SMALL, LIGHTGRAY);
    y += LINE;

    snprintf(buf, sizeof(buf), "Wall Time: %.2f s",
             (logger::elapsed() - wall_time_offset_ms) / 1000.0f);
    DrawText(buf, PAD, y, SMALL, LIGHTGRAY);
    y += LINE;

    snprintf(buf, sizeof(buf), "Time Loss: %.2f s",
             ((logger::elapsed() - wall_time_offset_ms) / 1000.0f) - snapshot.sim_time);
    Color time_color = (((logger::elapsed() - wall_time_offset_ms) / 1000.0f) - snapshot.sim_time) > 2.9f ? RED : LIGHTGRAY;
    DrawText(buf, PAD, y, SMALL, time_color);
    y += LINE;

    snprintf(buf, sizeof(buf), "Bodies: %d", (int)snapshot.bodies.size());
    DrawText(buf, PAD, y, SMALL, LIGHTGRAY);
    y += LINE + 4;

    // ── NT4 status ────────────────────────────────────────────────────────
    Color dot_col = nt_connected ? GREEN : RED;
    DrawCircle(PAD + 6, y + 6, 6, dot_col);
    DrawText(nt_connected ? "NT4  connected" : "NT4  disconnected",
             PAD + 18, y, SMALL, nt_connected ? GREEN : RED);
    if (nt_connected)
    {
        if (nt_staleness_ms < 0)
        {
            snprintf(buf, sizeof(buf), "  (no data yet)");
            DrawText(buf, PAD + 140, y, SMALL, ORANGE);
        }
        else if (nt_staleness_ms > 100.0f)
        {
            snprintf(buf, sizeof(buf), "   %.1f ms", nt_staleness_ms);
            DrawText(buf, PAD + 140, y, SMALL, RED);
        }
        else if (nt_staleness_ms > 20.0f)
        {
            snprintf(buf, sizeof(buf), "   %.1f ms", nt_staleness_ms);
            DrawText(buf, PAD + 140, y, SMALL, YELLOW);
        }
        else
        {
            snprintf(buf, sizeof(buf), "  %.1f ms", nt_staleness_ms);
            DrawText(buf, PAD + 140, y, SMALL,
                     GREEN );
        }
    }
    y += LINE + 6;

    // ── Intake / Shooter HUD ──────────────────────────────────────────────
    if (snapshot.intake_max_capacity > 0)
    {
        // Piece counter with coloured pip squares
        snprintf(buf, sizeof(buf), "Hopper: %d / %d",
                 snapshot.intake_held, snapshot.intake_max_capacity);
        DrawText(buf, PAD, y, SMALL, WHITE);

        
        float frac2 = (float)snapshot.intake_held/ (float)snapshot.intake_max_capacity;
        Color intake_c = GREEN;
        if(snapshot.intake_held == snapshot.intake_max_capacity){
            intake_c = BLUE;
        }
        DrawBar(PAD + 150, y + 2, 80, LINE - 6,
                frac2,
                intake_c,
                {40, 40, 40, 200});
        y += LINE + 2;

        // ── Shooter status ────────────────────────────────────────────────
        DrawText("Shooter:", PAD, y, SMALL, WHITE);
        if (snapshot.shooter_armed)
        {
            DrawText("ACTIVE", PAD + 72, y, SMALL, ORANGE);
        }
        else if (snapshot.intake_held > 0)
        {
            DrawText("READY", PAD + 72, y, SMALL, GREEN);
        }
        else
        {
            DrawText("EMPTY", PAD + 72, y, SMALL, Color{80, 80, 80, 200});
        }
        y += LINE + 2;
    }

    // ── Motor telemetry (robot body only) ─────────────────────────────────
    if (snapshot.robot_index < 0 ||
        snapshot.robot_index >= (int)snapshot.bodies.size())
        return;

    const BodySnapshot &robot = snapshot.bodies[snapshot.robot_index];
    if (robot.motors.empty())
        return;

    DrawText("Motors", PAD, y, SMALL, WHITE);
    y += LINE;

    for (int i = 0; i < (int)robot.motors.size(); ++i)
    {
        const MotorSnapshot &m = robot.motors[i];

        snprintf(buf, sizeof(buf), "[%d] %+6.1f rad/s", i, m.omega);
        DrawText(buf, PAD, y, SMALL, LIGHTGRAY);

        constexpr float FREE_SPEED = 608.0f; // Kraken, display-only
        float frac = std::clamp(m.omega / FREE_SPEED, -1.0f, 1.0f);
        DrawBar(PAD + 150, y + 2, 80, LINE - 6,
                frac,
                m.slipping ? RED : SKYBLUE,
                {40, 40, 40, 200});

        if (m.slipping)
            DrawText("SLIP", PAD + 240, y, SMALL, RED);

        y += LINE;
    }

    DrawText("Force Vectors:", PAD, y, SMALL, WHITE);
    y += LINE;

    for (int i = 0; i < (int)robot.motors.size(); ++i)
    {
        const MotorSnapshot &m = robot.motors[i];
        snprintf(buf, sizeof(buf), "[%d] N:%.0f F:%.0f", i, m.normal_force, m.tractive_force);
        DrawText(buf, PAD, y, SMALL, LIGHTGRAY);
        y += LINE;
    }
}