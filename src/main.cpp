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

// ─────────────────────────────────────────────────────────────────────────────
// CLI
// ─────────────────────────────────────────────────────────────────────────────

struct Args
{
    std::string scene;
    std::string nt_host = "127.0.0.1";
    int nt_port = 5810;
    float dt = 1.0f / 500.0f; // 500 Hz physics
    float speed = 1.0f;
    int target_fps = 60;
};

static void PrintUsage(const char *argv0)
{
    std::cout << "Usage: " << argv0 << " --scene <path> [options]\n"
                                       "\n"
                                       "  --scene  <path>      Scene JSON to load (required)\n"
                                       "  --nt     <host:port> NT4 server          (default: 127.0.0.1:5810)\n"
                                       "  --dt     <seconds>   Physics timestep    (default: 0.002)\n"
                                       "  --speed  <factor>    Sim speed multiplier(default: 1.0)\n"
                                       "  --fps    <target>    Target render FPS    (default: 60, zero: unlimited)\n"
                                       "\n";
}

static Args ParseArgs(int argc, char *argv[])
{
    Args args;
    for (int i = 1; i < argc; ++i)
    {
        if (!strcmp(argv[i], "--scene") && i + 1 < argc)
        {
            args.scene = argv[++i];
        }
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
        {
            args.dt = std::stof(argv[++i]);
        }
        else if (!strcmp(argv[i], "--speed") && i + 1 < argc)
        {
            args.speed = std::stof(argv[++i]);
        }
        else if (!strcmp(argv[i], "--fps") && i + 1 < argc)
        {
            args.target_fps = std::stoi(argv[++i]);
        }
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h"))
        {
            PrintUsage(argv[0]);
            std::exit(0);
        }
    }
    return args;
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
        args.target_fps = 90; // raylib max

    if (args.scene.empty())
    {
        PrintUsage(argv[0]);
        LOG_ERROR("main: --scene is required");
        return 1;
    }

    LOG_INFO("main: scene=%s  nt=%s:%d  dt=%.4f  speed=%.2f",
             args.scene.c_str(), args.nt_host.c_str(), args.nt_port,
             args.dt, args.speed);

    // ── 1. Renderer first (OpenGL context required before LoadModel) ──────
    Renderer renderer;
    renderer.Init(1280, 720, "FRC Sim", args.target_fps);

    // ── 2. Motor registry ─────────────────────────────────────────────────
    MotorRegistry motors;
    motors.LoadFromDirectory("assets/motors");

    // ── 3. Load scene ─────────────────────────────────────────────────────
    // LoadScene now returns a SceneData struct that also carries optional
    // IntakeDef / ShooterDef attached to the robot body's entry.
    SceneData scene = LoadScene(args.scene, motors);
    if (scene.bodies.empty())
    {
        LOG_ERROR("main: scene loaded no bodies — check path and JSON");
        renderer.Shutdown();
        return 1;
    }

    // ── 4. Physics world ──────────────────────────────────────────────────
    SimWorld world;
    world.Init();

    // ── 5. Spawn bodies ───────────────────────────────────────────────────
    int robot_motor_count = 0;

    for (int i = 0; i < (int)scene.bodies.size(); ++i)
    {
        auto &req = scene.bodies[i];

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

    // ── 6. Mechanism system (optional) ────────────────────────────────────
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

    // ── 7. Force applicator + sim loop ────────────────────────────────────
    ForceApplicator forces(world, motors, world.GetContactListener());
    SimLoop sim(world, &forces, mechanisms.get(), args.dt, args.speed);
    sim.Start();

    // ── 8. NT client ──────────────────────────────────────────────────────
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

    // ── 9. Main loop ──────────────────────────────────────────────────────
    while (!renderer.ShouldClose())
    {
        WorldSnapshot snapshot = sim.GetSnapshot();

        // Populate mechanism state into snapshot for HUD
        if (mechanisms)
        {
            snapshot.intake_held = mechanisms->HeldCount();
            snapshot.intake_max_capacity = scene.intake.max_capacity;
        }

        nt.Tick(snapshot);
        renderer.DrawFrame(snapshot, nt.IsConnected(),
                           sim.MeasuredHz(), sim.TargetHz());
    }

    // ── 10. Shutdown ──────────────────────────────────────────────────────
    sim.Stop();
    nt.Shutdown();
    renderer.Shutdown();

    LOG_INFO("main: clean exit");
    return 0;
}
