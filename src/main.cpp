#include "io/EasyLog.h"
#include "core/MotorRegistry.h"
#include "core/SceneLoader.h"
#include "core/SimWorld.h"
#include "core/SimLoop.h"
#include "physics/ForceApplicator.h"
#include "physics/ContactListener.h"
#include "physics/MechanismSystem.h"
#include "render/Renderer.h"
#include "render/BodyDraw.h"
#include "io/NTClient.h"

#include <string>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// CLI
// ─────────────────────────────────────────────────────────────────────────────

struct Args
{
    std::string scene;
    std::string nt_host = "127.0.0.1";
    int nt_port = 5810;
    float dt = 1.0f / 500.0f;
    float speed = 1.0f;
    int target_fps = 60;
    int width = 1280;
    int height = 720;
    bool wireframe = false;
};

static void PrintUsage(const char *argv0)
{
    std::cout << "Usage: " << argv0 << " --scene <path> [options]\n"
                                       "\n"
                                       "  --scene  <path>      Scene JSON to load (required)\n"
                                       "  --nt     <host:port> NT4 server          (default: 127.0.0.1:5810)\n"
                                       "  --dt     <seconds>   Physics timestep    (default: 0.002)\n"
                                       "  --speed  <factor>    Sim speed multiplier(default: 1.0)\n"
                                       "  --fps    <target>    Target render FPS    (default: 60)\n"
                                       "  --w      <width>     Window width         (default: 1280)\n"
                                       "  --h      <height>    Window height        (default: 720)\n"
                                       "  --wireframe          Enable wireframe overlay\n"
                                       "\n";
}

static Args ParseArgs(int argc, char *argv[])
{
    Args args;
    for (int i = 1; i < argc; ++i)
    {
        if (!strcmp(argv[i], "--scene") && i + 1 < argc)
            args.scene = argv[++i];
        else if (!strcmp(argv[i], "--nt") && i + 1 < argc)
        {
            std::string s = argv[++i];
            auto colon = s.rfind(':');
            if (colon != std::string::npos)
            {
                args.nt_host = s.substr(0, colon);
                args.nt_port = std::stoi(s.substr(colon + 1));
            }
            else
            {
                args.nt_host = s;
            }
        }
        else if (!strcmp(argv[i], "--dt") && i + 1 < argc)
            args.dt = std::stof(argv[++i]);
        else if (!strcmp(argv[i], "--speed") && i + 1 < argc)
            args.speed = std::stof(argv[++i]);
        else if (!strcmp(argv[i], "--fps") && i + 1 < argc)
            args.target_fps = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--w") && i + 1 < argc)
            args.width = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--h") && i + 1 < argc)
            args.height = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--wireframe"))
            args.wireframe = true;
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h"))
        {
            PrintUsage(argv[0]);
            std::exit(0);
        }
    }
    return args;
}

// ─────────────────────────────────────────────────────────────────────────────
// Loading screen
// ─────────────────────────────────────────────────────────────────────────────
//
// CAD-software aesthetic: dark engineering theme, subtle grid, monospaced
// technical readouts, clean progress bar with step + item counters.
// ─────────────────────────────────────────────────────────────────────────────

struct LoadCtx
{
    // Current phase
    const char *phase = "";  // e.g. "LOADING MOTORS"
    const char *detail = ""; // e.g. "kraken_x60.json"
    int cur = 0;             // items done in this phase
    int total = 0;           // total items in this phase (0 = indeterminate)

    // Overall progress  0.0 – 1.0
    float overall = 0.0f;

    // Elapsed seconds (set by DrawLoadingFrame)
    float elapsed = 0.0f;

    // Scene name shown at top
    std::string scene_name;
};

static void DrawLoadingFrame(LoadCtx &ctx)
{
    // Accumulate elapsed time using raylib's frame time
    ctx.elapsed += GetFrameTime();

    int sw = GetScreenWidth();
    int sh = GetScreenHeight();

    // ── Palette ───────────────────────────────────────────────────────────
    Color bg = {14, 16, 20, 255};
    Color grid_col = {28, 32, 40, 255};
    Color panel_bg = {20, 23, 30, 255};
    Color panel_edge = {38, 44, 56, 255};
    Color accent = {64, 160, 255, 255}; // engineering blue
    Color accent_dim = {32, 80, 128, 255};
    Color text_bright = {220, 228, 240, 255};
    Color text_mid = {120, 132, 152, 255};
    Color text_dim = {60, 68, 84, 255};
    Color ok_green = {60, 200, 120, 255};

    BeginDrawing();
    ClearBackground(bg);

    // ── Background grid (subtle engineering paper) ────────────────────────
    int grid_spacing = 32;
    for (int x = 0; x < sw; x += grid_spacing)
        DrawLine(x, 0, x, sh, grid_col);
    for (int y = 0; y < sh; y += grid_spacing)
        DrawLine(0, y, sw, y, grid_col);

    // Fade grid at edges with overdraw rectangles
    DrawRectangleGradientH(0, 0, 120, sh, bg, {14, 16, 20, 0});
    DrawRectangleGradientH(sw - 120, 0, 120, sh, {14, 16, 20, 0}, bg);
    DrawRectangleGradientV(0, 0, sw, 80, bg, {14, 16, 20, 0});
    DrawRectangleGradientV(0, sh - 80, sw, 80, {14, 16, 20, 0}, bg);

    // ── Central panel ─────────────────────────────────────────────────────
    int pw = 560;
    int ph = 280;
    int px = sw / 2 - pw / 2;
    int py = sh / 2 - ph / 2 - 10;

    // Panel shadow
    DrawRectangle(px + 4, py + 4, pw, ph, {0, 0, 0, 80});
    // Panel fill
    DrawRectangle(px, py, pw, ph, panel_bg);
    // Panel border
    DrawRectangleLines(px, py, pw, ph, panel_edge);
    // Accent top bar
    DrawRectangle(px, py, pw, 3, accent);

    // Corner tick marks (CAD aesthetic)
    int tick = 8;
    DrawLine(px - 1, py - 1, px + tick, py - 1, panel_edge);
    DrawLine(px - 1, py - 1, px - 1, py + tick, panel_edge);
    DrawLine(px + pw - tick, py - 1, px + pw + 1, py - 1, panel_edge);
    DrawLine(px + pw + 1, py - 1, px + pw + 1, py + tick, panel_edge);
    DrawLine(px - 1, py + ph - tick, px - 1, py + ph + 1, panel_edge);
    DrawLine(px - 1, py + ph + 1, px + tick, py + ph + 1, panel_edge);
    DrawLine(px + pw - tick, py + ph + 1, px + pw + 1, py + ph + 1, panel_edge);
    DrawLine(px + pw + 1, py + ph - tick, px + pw + 1, py + ph + 1, panel_edge);

    // ── App title ─────────────────────────────────────────────────────────
    int title_y = py + 20;
    const char *title = "FRC SIM 3D";
    int title_sz = 28;
    int title_x = px + pw / 2 - MeasureText(title, title_sz) / 2;
    DrawText(title, title_x, title_y, title_sz, text_bright);

    // Version / scene tag line
    if (!ctx.scene_name.empty())
    {
        char scene_label[128];
        snprintf(scene_label, sizeof(scene_label), "SCENE  %s", ctx.scene_name.c_str());
        int sl_x = px + pw / 2 - MeasureText(scene_label, 14) / 2;
        DrawText(scene_label, sl_x, title_y + 36, 14, text_mid);
    }

    // Divider
    DrawLine(px + 20, py + 74, px + pw - 20, py + 74, panel_edge);

    // ── Phase label ───────────────────────────────────────────────────────
    int phase_y = py + 86;
    DrawText(ctx.phase, px + 24, phase_y, 13, accent);

    // ── Detail + counter ──────────────────────────────────────────────────
    int detail_y = phase_y + 22;
    if (ctx.detail && ctx.detail[0])
    {
        // Truncate detail string if too long for panel
        char detail_buf[64];
        snprintf(detail_buf, sizeof(detail_buf), "%.60s", ctx.detail);
        DrawText(detail_buf, px + 24, detail_y, 13, text_mid);
    }

    // Item counter (right-aligned inside panel)
    if (ctx.total > 0)
    {
        char counter[32];
        snprintf(counter, sizeof(counter), "%d / %d", ctx.cur, ctx.total);
        int cx = px + pw - 24 - MeasureText(counter, 13);
        DrawText(counter, cx, detail_y, 13, ctx.cur == ctx.total ? ok_green : text_mid);
    }

    // ── Step progress bar (item-level) ────────────────────────────────────
    int bar_margin = 24;
    int bar_y = py + 142;
    int bar_w = pw - bar_margin * 2;
    int bar_h = 6;
    int bar_x = px + bar_margin;

    float step_frac = (ctx.total > 0)
                          ? std::min(1.0f, (float)ctx.cur / ctx.total)
                          : 0.0f;

    DrawRectangle(bar_x, bar_y, bar_w, bar_h, {30, 36, 48, 255});
    if (step_frac > 0.0f)
        DrawRectangle(bar_x, bar_y, (int)(bar_w * step_frac), bar_h, accent);
    DrawRectangleLines(bar_x, bar_y, bar_w, bar_h, panel_edge);

    // Glow on fill edge
    if (step_frac > 0.01f && step_frac < 1.0f)
    {
        int fill_end = bar_x + (int)(bar_w * step_frac);
        DrawRectangle(fill_end - 2, bar_y - 1, 3, bar_h + 2, {180, 220, 255, 120});
    }

    // ── Overall progress bar ──────────────────────────────────────────────
    int ov_y = bar_y + 22;
    int ov_h = 3;

    DrawRectangle(bar_x, ov_y, bar_w, ov_h, {24, 28, 38, 255});
    DrawRectangle(bar_x, ov_y, (int)(bar_w * ctx.overall), ov_h, accent_dim);
    DrawText("OVERALL", bar_x, ov_y + 7, 10, text_dim);

    char ov_pct[16];
    snprintf(ov_pct, sizeof(ov_pct), "%.0f%%", ctx.overall * 100.0f);
    DrawText(ov_pct, bar_x + bar_w - MeasureText(ov_pct, 10), ov_y + 7, 10, text_dim);

    // ── Animated spinner (three dots cycling) ────────────────────────────
    int spin_y = ov_y + 28;
    int dot_r = 3;
    int dot_gap = 14;
    int n_dots = 5;
    int dots_total_w = (n_dots - 1) * dot_gap;
    int dots_x = px + pw / 2 - dots_total_w / 2;

    for (int d = 0; d < n_dots; ++d)
    {
        float phase = ctx.elapsed * 2.5f - d * 0.25f;
        float brightness = 0.3f + 0.7f * (0.5f + 0.5f * sinf(phase * 3.14159f));
        Color dot_col = {
            (unsigned char)(accent.r * brightness),
            (unsigned char)(accent.g * brightness),
            (unsigned char)(accent.b * brightness),
            255};
        DrawCircle(dots_x + d * dot_gap, spin_y, dot_r, dot_col);
    }

    // ── Bottom status bar ─────────────────────────────────────────────────
    int status_y = py + ph + 14;
    char elapsed_buf[32];
    snprintf(elapsed_buf, sizeof(elapsed_buf), "%.1fs", ctx.elapsed);
    DrawText(elapsed_buf, px, status_y, 11, text_dim);

    // Right: build info
    const char *build_tag = "FRC Sim 3d by Arin J BUILD " __DATE__;
    DrawText(build_tag, px + pw - MeasureText(build_tag, 11), status_y, 11, text_dim);

    EndDrawing();
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char *argv[])
{
    logger::init();

    Args args = ParseArgs(argc, argv);

    if (args.dt <= 0.0f)
        args.dt = 1.0f / 500.0f;
    if (args.speed <= 0.0f)
        args.speed = 1.0f;
    if (args.target_fps < 0)
        args.target_fps = 60;
    if (args.target_fps > 90)
        args.target_fps = 90;
    if (args.width <= 0 || args.height <= 0)
    {
        LOG_ERROR("main: bad window dimensions");
        return 1;
    }
    if (args.scene.empty())
    {
        PrintUsage(argv[0]);
        LOG_ERROR("main: --scene is required");
        return 1;
    }

    LOG_INFO("main: scene=%s  nt=%s:%d  dt=%.4f  speed=%.2f",
             args.scene.c_str(), args.nt_host.c_str(), args.nt_port,
             args.dt, args.speed);

    // ── 1. Renderer ───────────────────────────────────────────────────────
    Renderer renderer;
    renderer.Init(args.width, args.height, "FRC Sim", args.target_fps);
    renderer.SetWireframe(args.wireframe);

    // Extract scene name for display (filename without extension)
    std::string scene_display = args.scene;
    {
        auto slash = scene_display.rfind('/');
        if (slash != std::string::npos)
            scene_display = scene_display.substr(slash + 1);
        auto dot = scene_display.rfind('.');
        if (dot != std::string::npos)
            scene_display = scene_display.substr(0, dot);
        // uppercase
        for (auto &c : scene_display)
            c = (char)toupper(c);
    }

    LoadCtx lctx;
    lctx.scene_name = scene_display;

    // ── 2. Motor registry ─────────────────────────────────────────────────
    lctx.phase = "LOADING MOTORS";
    lctx.detail = "assets/motors/";
    lctx.cur = 0;
    lctx.total = 0; // unknown count — indeterminate
    lctx.overall = 0.05f;
    DrawLoadingFrame(lctx);

    MotorRegistry motors;
    motors.LoadFromDirectory("assets/motors");

    // ── 3. Load scene JSON ────────────────────────────────────────────────
    lctx.phase = "PARSING SCENE";
    lctx.detail = args.scene.c_str();
    lctx.cur = 0;
    lctx.total = 0;
    lctx.overall = 0.10f;
    DrawLoadingFrame(lctx);

    SceneData scene = LoadScene(args.scene, motors);
    if (scene.bodies.empty())
    {
        LOG_ERROR("main: scene loaded no bodies — check path and JSON");
        renderer.Shutdown();
        return 1;
    }

    // ── 4. Physics world ──────────────────────────────────────────────────
    lctx.phase = "INITIALISING PHYSICS";
    lctx.detail = "Jolt Physics engine";
    lctx.cur = 0;
    lctx.total = 0;
    lctx.overall = 0.20f;
    DrawLoadingFrame(lctx);

    SimWorld world;
    world.Init();

    // ── 5. Spawn bodies ───────────────────────────────────────────────────
    int robot_motor_count = 0;
    int total_bodies = (int)scene.bodies.size();

    for (int i = 0; i < total_bodies; ++i)
    {
        auto &req = scene.bodies[i];

        lctx.phase = "LOADING MODELS";
        lctx.detail = req.def.name.c_str();
        lctx.cur = i;
        lctx.total = total_bodies;
        lctx.overall = 0.25f + 0.55f * ((float)i / total_bodies);
        DrawLoadingFrame(lctx);

        PreloadMesh(&req.def);

        auto id = world.SpawnBody(req.def,
                                  req.position.data(),
                                  req.orientation.data());
        if (id.IsInvalid())
        {
            LOG_WARN("main: body '%s' failed to spawn", req.def.name.c_str());
            continue;
        }

        int spawned_idx = world.BodyCount() - 1;
        if (req.role == "robot")
        {
            world.SetRobotIndex(spawned_idx);
            robot_motor_count = (int)req.def.motors.size();
            LOG_INFO("main: robot body at index %d (%d motors)",
                     spawned_idx, robot_motor_count);
        }
    }

    // Show completion of model loading
    lctx.cur = total_bodies;
    lctx.overall = 0.80f;
    DrawLoadingFrame(lctx);

    // ── 6. Mechanism system ───────────────────────────────────────────────
    lctx.phase = "BUILDING MECHANISMS";
    lctx.detail = scene.has_intake ? "intake + shooter" : "none";
    lctx.cur = 0;
    lctx.total = 0;
    lctx.overall = 0.85f;
    DrawLoadingFrame(lctx);

    std::unique_ptr<MechanismSystem> mechanisms;
    if (scene.has_intake)
    {
        mechanisms = std::make_unique<MechanismSystem>(
            world, scene.intake, scene.shooter);
        LOG_INFO("main: MechanismSystem created  intake_cap=%d",
                 scene.intake.max_capacity);
    }
    else
    {
        LOG_INFO("main: no intake/shooter defined in scene");
    }

    // ── 7. Sim loop + NT ──────────────────────────────────────────────────
    lctx.phase = "STARTING SIMULATION";
    lctx.detail = "physics thread + NT4 client";
    lctx.cur = 0;
    lctx.total = 0;
    lctx.overall = 0.92f;
    DrawLoadingFrame(lctx);

    ForceApplicator forces(world, motors, world.GetContactListener());
    SimLoop sim(world, &forces, mechanisms.get(), args.dt, args.speed);
    renderer.SetWallTimeOffset(logger::elapsed());
    sim.Start();

    // ── 8. NT client ──────────────────────────────────────────────────────
    lctx.phase = "CONNECTING NT4";
    char nt_addr[64];
    snprintf(nt_addr, sizeof(nt_addr), "%s:%d", args.nt_host.c_str(), args.nt_port);
    lctx.detail = nt_addr;
    lctx.cur = 0;
    lctx.total = 0;
    lctx.overall = 0.97f;
    DrawLoadingFrame(lctx);

    NTClient nt;
    if (robot_motor_count > 0)
    {
        nt.Init(args.nt_host, args.nt_port, world, robot_motor_count,
                mechanisms.get());
    }
    else
    {
        LOG_WARN("main: no robot body found — NT client not started");
    }

    // Final frame — complete
    lctx.phase = "READY";
    lctx.detail = "";
    lctx.cur = 1;
    lctx.total = 1;
    lctx.overall = 1.0f;
    DrawLoadingFrame(lctx);

    // ── 9. Main loop ──────────────────────────────────────────────────────
    while (!renderer.ShouldClose())
    {
        WorldSnapshot snapshot = sim.GetSnapshot();

        if (mechanisms)
        {
            snapshot.intake_held = mechanisms->HeldCount();
            snapshot.intake_max_capacity = scene.intake.max_capacity;
        }

        nt.Tick(snapshot, GetFrameTime()); //not physics dt
        renderer.DrawFrame(snapshot, nt.IsConnected(),
                           sim.MeasuredHz(), sim.TargetHz(), nt.Ping());
    }

    // ── 10. Shutdown ──────────────────────────────────────────────────────
    sim.Stop();
    nt.Shutdown();
    renderer.Shutdown();

    LOG_INFO("main: clean exit");
    return 0;
}