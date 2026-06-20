#include "io/EasyLog.h"
#include "core/MotorRegistry.h"
#include "core/SceneLoader.h"
#include "core/BodyLoader.h"
#include "core/SimWorld.h"
#include "core/SimLoop.h"
#include "physics/ForceApplicator.h"
#include "physics/ContactListener.h"
#include "physics/MechanismSystem.h"
#include "core/ScoreTracker.h"
#include "core/ScoringDef.h"
#include "render/Renderer.h"
#include "render/BodyDraw.h"
#include "render/MeshCache.h"
#include "io/NTClient.h"
#include "io/RaycastDef.h"
#include "io/Raycaster.h"

#include <sokol_app.h>
#include <sokol_gfx.h>
#include <sokol_glue.h>
#include <sokol_time.h>
#include <sokol_log.h>
#include <sokol_debugtext.h>

#include <string>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <random>

// ── CLI args ──────────────────────────────────────────────────────────────────

struct Args {
    std::string scene;
    struct RobotArg {
        std::string def_path;
        std::string nt_host = "127.0.0.1";
        int nt_port = 5810;
    };
    std::vector<RobotArg> robots;
    float dt = 1.0f / 500.0f;
    float speed = 1.0f;
    int substeps = 2;
    int threads = 0;
    int target_fps = 60;
    int width = 1280;
    int height = 720;
    bool wireframe = false;
    bool stream = false;
    std::string stream_host = "127.0.0.1";
    int stream_port = 5000;
    int stream_fps = 30;
    std::string raycast_path;
    std::string backend = "gl";
};

static Args g_args;

static void PrintUsage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " --scene <path> --robot <def@host:port> [--robot ...] [options]\n"
                 "\n"
                 "  --scene  <path>              Scene JSON (required)\n"
                 "  --robot  <def@host:port>     Add a robot (repeatable, up to 6)\n"
                 "  --dt     <seconds>           Physics timestep    (default: 0.002)\n"
                 "  --substeps <n>               Jolt collision steps per tick (default: 2)\n"
                 "  --speed  <factor>            Sim speed multiplier(default: 1.0)\n"
                 "  --fps    <target>            Target render FPS   (default: 60)\n"
                 "  --w      <width>             Window width        (default: 1280)\n"
                 "  --h      <height>            Window height       (default: 720)\n"
                 "  --threads <n>                Jolt worker threads (default: auto)\n"
                 "  --wireframe                  Enable wireframe overlay\n"
                 "  --backend <gl|vulkan>        Graphics backend    (default: gl)\n"
                 "  --stream <port>              Streaming port      (default: 5000)\n"
                 "  --stream-fps <fps>           Stream frame rate   (default: 30)\n"
                 "  --raycast <path.json>        Raycast sensor definitions\n";
}

static Args ParseArgs(int argc, char* argv[]) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--scene") && i+1 < argc) a.scene = argv[++i];
        else if (!strcmp(argv[i], "--robot") && i+1 < argc) {
            std::string s = argv[++i];
            Args::RobotArg ra;
            auto at = s.rfind('@');
            if (at != std::string::npos) {
                ra.def_path = s.substr(0, at);
                std::string addr = s.substr(at+1);
                auto colon = addr.rfind(':');
                if (colon != std::string::npos) {
                    ra.nt_host = addr.substr(0, colon);
                    ra.nt_port = std::stoi(addr.substr(colon+1));
                } else ra.nt_host = addr;
            } else ra.def_path = s;
            if (a.robots.size() < 6) a.robots.push_back(std::move(ra));
            else LOG_WARN("main: max 6 robots");
        }
        else if (!strcmp(argv[i], "--dt") && i+1 < argc) a.dt = std::stof(argv[++i]);
        else if (!strcmp(argv[i], "--substeps") && i+1 < argc) a.substeps = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--speed") && i+1 < argc) a.speed = std::stof(argv[++i]);
        else if (!strcmp(argv[i], "--fps") && i+1 < argc) a.target_fps = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--w") && i+1 < argc) a.width = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--h") && i+1 < argc) a.height = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--threads") && i+1 < argc) a.threads = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--wireframe")) a.wireframe = true;
        else if (!strcmp(argv[i], "--backend") && i+1 < argc) a.backend = argv[++i];
        else if (!strcmp(argv[i], "--raycast") && i+1 < argc) a.raycast_path = argv[++i];
        else if (!strcmp(argv[i], "--stream")) {
            a.stream = true;
            if (i+1 < argc) a.stream_port = std::stoi(argv[++i]);
        }
        else if (!strcmp(argv[i], "--stream-fps") && i+1 < argc) a.stream_fps = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            PrintUsage(argv[0]); std::exit(0);
        }
    }
    return a;
}

// ── Global app state ──────────────────────────────────────────────────────────

struct App {
    Renderer renderer;
    SimWorld world;
    std::unique_ptr<ForceApplicator> forces;
    std::unique_ptr<SimLoop> sim;
    std::vector<std::unique_ptr<NTClient>> nt_clients;
    std::vector<std::unique_ptr<Raycaster>> raycasters;
    std::vector<int> robot_motor_counts;
    std::vector<std::unique_ptr<MechanismSystem>> all_mechanisms;
    SceneData scene;
    ScoreTracker score_tracker;
    MotorRegistry motors;
    std::vector<BodyDef> robot_defs;
    float wall_time_offset = 0;
    bool reset_just_happened = false;
    uint64_t frame_stamp = 0;
    std::function<void()> do_reset;
};

static App* g_app = nullptr;

static void ApplySpawnRandomization(const RobotSpawn& spawn, float pos[3], float rot[4]) {
    float hx = spawn.randomize_half_extents[0];
    float hz = spawn.randomize_half_extents[2];
    if (hx > 0.0f || hz > 0.0f) {
        thread_local std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<float> dx(-hx, hx);
        std::uniform_real_distribution<float> dz(-hz, hz);
        pos[0] = spawn.position[0] + dx(rng);
        pos[2] = spawn.position[2] + dz(rng);
    }
    if (spawn.randomize_rotation) {
        thread_local std::mt19937 rng_rot(std::random_device{}());
        std::uniform_real_distribution<float> da(0.0f, 2.0f * 3.14159265f);
        float angle = da(rng_rot);
        JPH::Quat base_q(rot[0], rot[1], rot[2], rot[3]);
        JPH::Quat yaw_q = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), angle);
        JPH::Quat result = yaw_q * base_q;
        rot[0] = result.GetX(); rot[1] = result.GetY();
        rot[2] = result.GetZ(); rot[3] = result.GetW();
    }
}

// ── sokol_app callbacks ───────────────────────────────────────────────────────

static void init_cb() {
    stm_setup();

    sg_desc sgdesc = {};
    sgdesc.environment = sglue_environment();
    sgdesc.logger.func = slog_func;
    sgdesc.buffer_pool_size = 1024;
    sgdesc.image_pool_size = 256;
    sgdesc.shader_pool_size = 32;
    sgdesc.pipeline_pool_size = 256;
    sgdesc.view_pool_size = 256;
    sg_setup(&sgdesc);
    if (!sg_isvalid()) {
        LOG_ERROR("sokol_gfx: failed to initialize");
        return;
    }
    sdtx_desc_t sdtx_desc = {};
    sdtx_desc.logger.func = slog_func;
    sdtx_setup(&sdtx_desc);

    g_app = new App();
    App& app = *g_app;

    app.renderer.Init(g_args.width, g_args.height, "FRC Sim 3D", g_args.target_fps);
    app.renderer.SetWireframe(g_args.wireframe);
    app.frame_stamp = stm_now();

    app.motors.LoadFromDirectory("assets/motors");

    app.scene = LoadScene(g_args.scene, app.motors);
    app.score_tracker.LoadZones(app.scene.scoring_zones);

    app.world.Init(g_args.threads);
    app.world.SetPhysicsDt(g_args.dt);
    app.world.SetCollisionSteps(g_args.substeps);

    int total_bodies = (int)app.scene.bodies.size() + (int)g_args.robots.size();

    for (int i = 0; i < (int)app.scene.bodies.size(); ++i) {
        auto& req = app.scene.bodies[i];
        PreloadMesh(&req.def, &app.renderer.GetMeshCache());
        auto id = app.world.SpawnBody(req.def, req.position.data(), req.orientation.data());
        if (id.IsInvalid())
            LOG_WARN("main: body '%s' failed to spawn", req.def.name.c_str());
    }

    app.robot_defs.clear();
    app.robot_defs.reserve(g_args.robots.size());

    for (int ri = 0; ri < (int)g_args.robots.size(); ++ri) {
        auto& ra = g_args.robots[ri];
        auto maybe = LoadBodyDef(ra.def_path, app.motors);
        if (!maybe) {
            LOG_ERROR("main: failed to load robot def: %s", ra.def_path.c_str());
            app.robot_motor_counts.push_back(0);
            app.all_mechanisms.push_back(nullptr);
            continue;
        }
        app.robot_defs.push_back(std::move(*maybe));
        BodyDef& def = app.robot_defs.back();

        float pos[3] = {0.0f, 0.051f, (float)ri * 1.5f};
        float rot[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        if (ri < (int)app.scene.robot_spawns.size()) {
            auto& rs = app.scene.robot_spawns[ri];
            pos[0]=rs.position[0]; pos[1]=rs.position[1]; pos[2]=rs.position[2];
            rot[0]=rs.orientation[0]; rot[1]=rs.orientation[1];
            rot[2]=rs.orientation[2]; rot[3]=rs.orientation[3];
        }
        if (ri < (int)app.scene.robot_spawns.size())
            ApplySpawnRandomization(app.scene.robot_spawns[ri], pos, rot);

        PreloadMesh(&def, &app.renderer.GetMeshCache());
        auto id = app.world.SpawnBody(def, pos, rot);
        if (id.IsInvalid()) {
            LOG_ERROR("main: robot[%d] '%s' failed to spawn", ri, def.name.c_str());
            app.robot_motor_counts.push_back(0);
            app.all_mechanisms.push_back(nullptr);
            continue;
        }
        int body_idx = app.world.BodyCount() - 1;
        app.world.AddRobotIndex(body_idx);
        app.robot_motor_counts.push_back((int)def.motors.size());

        if (ri < (int)app.scene.robot_spawns.size() && app.scene.robot_spawns[ri].has_mechanisms) {
            auto& rs = app.scene.robot_spawns[ri];
            app.all_mechanisms.push_back(std::make_unique<MechanismSystem>(
                app.world, rs.intake, rs.shooter, body_idx));
        } else {
            app.all_mechanisms.push_back(nullptr);
        }
    }

    std::vector<MechanismSystem*> mech_ptrs;
    for (auto& m : app.all_mechanisms) mech_ptrs.push_back(m.get());

    app.forces = std::make_unique<ForceApplicator>(app.world, app.motors, app.world.GetContactListener());
    app.sim = std::make_unique<SimLoop>(app.world, app.forces.get(), std::move(mech_ptrs),
                                        &app.score_tracker, g_args.dt, g_args.speed);
    app.sim->Start();

    auto do_reset = [&app]() {
        LOG_INFO("main: reset triggered via NT");
        app.sim->Stop();
        auto& bi = app.world.GetBodyInterface();
        std::vector<JPH::BodyID> to_remove;
        const auto& ri_vec = app.world.GetRobotIndices();
        for (int i = 0; i < app.world.BodyCount(); ++i) {
            bool is_robot = false;
            for (int ri : ri_vec) if (i == ri) { is_robot = true; break; }
            if (is_robot) continue;
            const BodyDef* def = app.world.GetBodyDef(i);
            if (!def || def->mass == 0.0f) continue;
            to_remove.push_back(app.world.GetBodyID(i));
        }
        for (auto id : to_remove) {
            bi.RemoveBody(id);
            bi.DestroyBody(id);
            app.world.MarkBodyRemoved(id);
        }
        for (auto& req : app.scene.bodies) {
            if (req.role != "gamepiece") continue;
            app.world.SpawnBody(req.def, req.position.data(), req.orientation.data());
        }
        for (int ri = 0; ri < (int)g_args.robots.size(); ++ri) {
            if (ri >= (int)app.world.GetRobotIndices().size()) break;
            int body_idx = app.world.GetRobotIndices()[ri];
            JPH::BodyID bid = app.world.GetBodyID(body_idx);
            if (bid.IsInvalid()) continue;
            float pos[3] = {0.0f, 0.3f, 0.0f};
            float rot[4] = {0.0f, 0.0f, 0.0f, 1.0f};
            if (ri < (int)app.scene.robot_spawns.size()) {
                auto& rs = app.scene.robot_spawns[ri];
                pos[0]=rs.position[0]; pos[1]=rs.position[1]; pos[2]=rs.position[2];
                rot[0]=rs.orientation[0]; rot[1]=rs.orientation[1];
                rot[2]=rs.orientation[2]; rot[3]=rs.orientation[3];
                ApplySpawnRandomization(rs, pos, rot);
            }
            bi.SetPositionAndRotation(bid,
                JPH::RVec3(pos[0], pos[1], pos[2]),
                JPH::Quat(rot[0], rot[1], rot[2], rot[3]),
                JPH::EActivation::Activate);
            bi.SetLinearVelocity(bid, JPH::Vec3::sZero());
            bi.SetAngularVelocity(bid, JPH::Vec3::sZero());
        }
        for (auto& mech : app.all_mechanisms)
            if (mech) mech->Reset();
        app.score_tracker.StartMatch();
        app.sim->Start();
        app.reset_just_happened = true;
        LOG_INFO("main: reset complete");
    };
    app.do_reset = do_reset;

    const auto& robot_indices = app.world.GetRobotIndices();
    int spawn_slot = 0;
    for (int ri = 0; ri < (int)g_args.robots.size(); ++ri) {
        if (app.robot_motor_counts[ri] == 0) continue;
        if (spawn_slot >= (int)robot_indices.size()) break;
        auto& ra = g_args.robots[ri];
        auto client = std::make_unique<NTClient>();
        client->Init(ra.nt_host, ra.nt_port, app.world,
            app.robot_motor_counts[ri], ri,
            app.all_mechanisms[ri].get(),
            app.scene.has_field_bounds,
            app.scene.field_half_x, app.scene.field_half_z,
            (ri == 0) ? do_reset : std::function<void()>{});
        ++spawn_slot;
        app.nt_clients.push_back(std::move(client));
        LOG_INFO("main: NT client[%d] -> %s:%d", ri, ra.nt_host.c_str(), ra.nt_port);
    }

    if (!g_args.raycast_path.empty()) {
        auto cfg = LoadRaycastConfig(g_args.raycast_path);
        if (cfg) {
            for (int ri = 0; ri < (int)app.nt_clients.size(); ++ri) {
                auto rc = std::make_unique<Raycaster>();
                rc->Init(*cfg, app.nt_clients[ri]->GetInst(), ri);
                app.raycasters.push_back(std::move(rc));
            }
        }
    }

    {
        std::vector<Raycaster*> rc_ptrs;
        for (auto& rc : app.raycasters) rc_ptrs.push_back(rc.get());
        app.renderer.SetRaycasters(std::move(rc_ptrs));
    }

    app.wall_time_offset = logger::elapsed();
    app.renderer.SetWallTimeOffset(app.wall_time_offset);

    if (g_args.stream)
        app.renderer.EnableStreaming(g_args.stream_port, g_args.stream_fps);

    sapp_show_keyboard(false);
}

static void frame_cb() {
    if (!g_app) return;

    App& app = *g_app;

    WorldSnapshot snapshot = app.sim->GetSnapshot();
    float frame_dt = stm_sec(stm_laptime(&app.frame_stamp));
    if (frame_dt > 0.1f) frame_dt = 0.1f;

    bool any_connected = false;
    float best_ping = -1.0f;
    for (auto& nt : app.nt_clients) {
        nt->Tick(snapshot, frame_dt);
        if (nt->IsConnected()) {
            any_connected = true;
            float p = nt->Ping();
            if (p >= 0) best_ping = p;
        }
        for (auto& rc : app.raycasters)
            rc->CastAndPublish(snapshot, app.world);
    }

    if (app.reset_just_happened) {
        app.reset_just_happened = false;
        snapshot = app.sim->GetSnapshot();
        for (auto& nt : app.nt_clients) nt->Tick(snapshot, frame_dt);
    }

    app.renderer.DrawFrame(snapshot, any_connected,
                           app.sim->MeasuredHz(), app.sim->TargetHz(), best_ping);
}

static void event_cb(const sapp_event* e) {
    if (!g_app) return;
    App& app = *g_app;

    app.renderer.HandleEvent(e);

    if (e->type == SAPP_EVENTTYPE_KEY_DOWN &&
        (e->key_code == SAPP_KEYCODE_LEFT_ALT || e->key_code == SAPP_KEYCODE_RIGHT_ALT) &&
        app.score_tracker.GetPhase() == MatchPhase::WAITING) {
        app.score_tracker.StartMatch();
    }
}

static void cleanup_cb() {
    if (!g_app) return;
    App& app = *g_app;
    app.sim->Stop();
    for (auto& nt : app.nt_clients) nt->Shutdown();
    app.renderer.Shutdown();
    sdtx_shutdown();
    sg_shutdown();
    delete g_app;
    g_app = nullptr;
    LOG_INFO("main: clean exit");
}

// ── sokol_main ────────────────────────────────────────────────────────────────

sapp_desc sokol_main(int argc, char* argv[]) {
    logger::init();
    g_args = ParseArgs(argc, argv);

    if (g_args.dt <= 0.0f) g_args.dt = 1.0f / 500.0f;
    if (g_args.speed <= 0.0f) g_args.speed = 1.0f;
    if (g_args.target_fps < 0) g_args.target_fps = 60;
    if (g_args.width <= 0 || g_args.height <= 0) {
        LOG_ERROR("main: bad window dimensions"); exit(1);
    }
    if (g_args.scene.empty()) {
        PrintUsage(argv[0]); LOG_ERROR("main: --scene is required"); exit(1);
    }

    sapp_desc desc = {};
    desc.init_cb = init_cb;
    desc.frame_cb = frame_cb;
    desc.event_cb = event_cb;
    desc.cleanup_cb = cleanup_cb;
    desc.width = g_args.width;
    desc.height = g_args.height;
    desc.window_title = "FRC Sim 3D";
    desc.sample_count = 4; // MSAA 4x
    desc.swap_interval = (g_args.target_fps > 0) ? 1 : 0;
    desc.logger.func = slog_func;

    return desc;
}
