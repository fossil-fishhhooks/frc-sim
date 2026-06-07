#include "io/EasyLog.h"
#include "core/MotorRegistry.h"
#include "core/SceneLoader.h"
#include "core/BodyLoader.h"
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
#include <vector>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// CLI
// ─────────────────────────────────────────────────────────────────────────────

struct Args
{
    std::string scene;

    struct RobotArg {
        std::string def_path;
        std::string nt_host = "127.0.0.1";
        int         nt_port = 5810;
    };
    std::vector<RobotArg> robots;  // one per --robot flag, up to 6

    float dt         = 1.0f / 500.0f;
    float speed      = 1.0f;
    int   target_fps = 60;
    int   width      = 1280;
    int   height     = 720;
    bool  wireframe  = false;


    bool        stream      = false;
    std::string stream_host = "127.0.0.1";
    int         stream_port = 5000;
    int         stream_fps  = 30;
};

static void PrintUsage(const char *argv0)
{
    std::cout << "Usage: " << argv0 << " --scene <path> --robot <def@host:port> [--robot ...] [options]\n"
                                       "\n"
                                       "  --scene  <path>              Scene JSON (required)\n"
                                       "  --robot  <def@host:port>     Add a robot (repeatable, up to 6)\n"
                                       "                               e.g. assets/defs/robot.json@10.9.55.2:5810\n"
                                       "  --dt     <seconds>           Physics timestep    (default: 0.002)\n"
                                       "  --speed  <factor>            Sim speed multiplier(default: 1.0)\n"
                                       "  --fps    <target>            Target render FPS   (default: 60)\n"
                                       "  --w      <width>             Window width        (default: 1280)\n"
                                       "  --h      <height>            Window height       (default: 720)\n"
                                       "  --wireframe                  Enable wireframe overlay\n"
                                       "  --stream <host:port>         Stream H.264 over UDP (default: 127.0.0.1:5000)\n"
                                       "  --stream-fps <fps>           Stream frame rate     (default: 30)\n"       
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
        else if (!strcmp(argv[i], "--robot") && i + 1 < argc)
        {
            std::string s = argv[++i];
            Args::RobotArg ra;
            auto at = s.rfind('@');
            if (at != std::string::npos) {
                ra.def_path = s.substr(0, at);
                std::string addr = s.substr(at + 1);
                auto colon = addr.rfind(':');
                if (colon != std::string::npos) {
                    ra.nt_host = addr.substr(0, colon);
                    ra.nt_port = std::stoi(addr.substr(colon + 1));
                } else {
                    ra.nt_host = addr;
                }
            } else {
                ra.def_path = s;
            }
            if (args.robots.size() < 6)
                args.robots.push_back(std::move(ra));
            else
                LOG_WARN("main: max 6 robots, ignoring extra --robot");
        }
        else if (!strcmp(argv[i], "--dt")    && i + 1 < argc) args.dt         = std::stof(argv[++i]);
        else if (!strcmp(argv[i], "--speed") && i + 1 < argc) args.speed      = std::stof(argv[++i]);
        else if (!strcmp(argv[i], "--fps")   && i + 1 < argc) args.target_fps = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--w")     && i + 1 < argc) args.width      = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--h")     && i + 1 < argc) args.height     = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--wireframe")) args.wireframe = true;
        else if (!strcmp(argv[i], "--stream"))
        {
            args.stream = true;
            if (i + 1 < argc && argv[i+1][0] != '-')
            {
                std::string s = argv[++i];
                auto colon = s.rfind(':');
                if (colon != std::string::npos) {
                    args.stream_host = s.substr(0, colon);
                    args.stream_port = std::stoi(s.substr(colon + 1));
                } else {
                    args.stream_host = s;
                }
            }
        }
        else if (!strcmp(argv[i], "--stream-fps") && i + 1 < argc)
            args.stream_fps = std::stoi(argv[++i]);
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h"))
        {
            PrintUsage(argv[0]);
            std::exit(0);
        }
    }
    return args;
}

// ─────────────────────────────────────────────────────────────────────────────
// Loading screen (unchanged from single-robot version)
// ─────────────────────────────────────────────────────────────────────────────

struct LoadCtx
{
    const char  *phase   = "";
    const char  *detail  = "";
    int          cur     = 0;
    int          total   = 0;
    float        overall = 0.0f;
    float        elapsed = 0.0f;
    std::string  scene_name;
};

static void DrawLoadingFrame(LoadCtx &ctx)
{
    ctx.elapsed += GetFrameTime();
    int sw = GetScreenWidth(), sh = GetScreenHeight();

    Color bg        = {14,  16,  20,  255};
    Color grid_col  = {28,  32,  40,  255};
    Color panel_bg  = {20,  23,  30,  255};
    Color panel_edge= {38,  44,  56,  255};
    Color accent    = {64, 160, 255,  255};
    Color accent_dim= {32,  80, 128,  255};
    Color text_bright={220,228, 240,  255};
    Color text_mid  = {120,132, 152,  255};
    Color text_dim  = { 60, 68,  84,  255};
    Color ok_green  = { 60,200, 120,  255};

    BeginDrawing();
    ClearBackground(bg);

    for (int x = 0; x < sw; x += 32) DrawLine(x, 0, x, sh, grid_col);
    for (int y = 0; y < sh; y += 32) DrawLine(0, y, sw, y, grid_col);
    DrawRectangleGradientH(0,     0, 120, sh, bg, {14,16,20,0});
    DrawRectangleGradientH(sw-120,0, 120, sh, {14,16,20,0}, bg);
    DrawRectangleGradientV(0, 0,     sw, 80,  bg, {14,16,20,0});
    DrawRectangleGradientV(0, sh-80, sw, 80,  {14,16,20,0}, bg);

    int pw=560, ph=280, px=sw/2-pw/2, py=sh/2-ph/2-10;
    DrawRectangle(px+4, py+4, pw, ph, {0,0,0,80});
    DrawRectangle(px, py, pw, ph, panel_bg);
    DrawRectangleLines(px, py, pw, ph, panel_edge);
    DrawRectangle(px, py, pw, 3, accent);

    int tk=8;
    DrawLine(px-1,py-1,px+tk,py-1,panel_edge); DrawLine(px-1,py-1,px-1,py+tk,panel_edge);
    DrawLine(px+pw-tk,py-1,px+pw+1,py-1,panel_edge); DrawLine(px+pw+1,py-1,px+pw+1,py+tk,panel_edge);
    DrawLine(px-1,py+ph-tk,px-1,py+ph+1,panel_edge); DrawLine(px-1,py+ph+1,px+tk,py+ph+1,panel_edge);
    DrawLine(px+pw-tk,py+ph+1,px+pw+1,py+ph+1,panel_edge); DrawLine(px+pw+1,py+ph-tk,px+pw+1,py+ph+1,panel_edge);

    const char *title = "FRC SIM 3D";
    DrawText(title, px+pw/2-MeasureText(title,28)/2, py+20, 28, text_bright);
    if (!ctx.scene_name.empty()) {
        char sl[128]; snprintf(sl,sizeof(sl),"SCENE  %s",ctx.scene_name.c_str());
        DrawText(sl, px+pw/2-MeasureText(sl,14)/2, py+56, 14, text_mid);
    }
    DrawLine(px+20, py+74, px+pw-20, py+74, panel_edge);

    DrawText(ctx.phase, px+24, py+86, 13, accent);
    if (ctx.detail && ctx.detail[0]) {
        char db[64]; snprintf(db,sizeof(db),"%.60s",ctx.detail);
        DrawText(db, px+24, py+108, 13, text_mid);
    }
    if (ctx.total > 0) {
        char ctr[32]; snprintf(ctr,sizeof(ctr),"%d / %d",ctx.cur,ctx.total);
        DrawText(ctr, px+pw-24-MeasureText(ctr,13), py+108, 13,
                 ctx.cur==ctx.total ? ok_green : text_mid);
    }

    int bx=px+24, bw=pw-48, by=py+142;
    float sf=(ctx.total>0)?std::min(1.0f,(float)ctx.cur/ctx.total):0.0f;
    DrawRectangle(bx,by,bw,6,{30,36,48,255});
    if(sf>0) DrawRectangle(bx,by,(int)(bw*sf),6,accent);
    DrawRectangleLines(bx,by,bw,6,panel_edge);
    if(sf>0.01f&&sf<1.0f) DrawRectangle(bx+(int)(bw*sf)-2,by-1,3,8,{180,220,255,120});

    int oy=by+22;
    DrawRectangle(bx,oy,bw,3,{24,28,38,255});
    DrawRectangle(bx,oy,(int)(bw*ctx.overall),3,accent_dim);
    DrawText("OVERALL",bx,oy+7,10,text_dim);
    char op[16]; snprintf(op,sizeof(op),"%.0f%%",ctx.overall*100);
    DrawText(op,bx+bw-MeasureText(op,10),oy+7,10,text_dim);

    int sy=oy+28, dg=14, nd=5, dx2=px+pw/2-(nd-1)*dg/2;
    for(int d=0;d<nd;++d){
        float ph2=ctx.elapsed*2.5f-d*0.25f;
        float br=0.3f+0.7f*(0.5f+0.5f*sinf(ph2*3.14159f));
        DrawCircle(dx2+d*dg,sy,3,{(unsigned char)(accent.r*br),(unsigned char)(accent.g*br),(unsigned char)(accent.b*br),255});
    }

    char eb[32]; snprintf(eb,sizeof(eb),"%.1fs",ctx.elapsed);
    DrawText(eb,px,py+ph+14,11,text_dim);
    const char *bt="FRC Sim 3d BUILD " __DATE__;
    DrawText(bt,px+pw-MeasureText(bt,11),py+ph+14,11,text_dim);
    EndDrawing();
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char *argv[])
{
    logger::init();

    Args args = ParseArgs(argc, argv);

    if (args.dt     <= 0.0f) args.dt     = 1.0f / 500.0f;
    if (args.speed  <= 0.0f) args.speed  = 1.0f;
    if (args.target_fps < 0) args.target_fps = 60;
    if (args.width <= 0 || args.height <= 0) {
        LOG_ERROR("main: bad window dimensions"); return 1;
    }
    if (args.scene.empty()) {
        PrintUsage(argv[0]); LOG_ERROR("main: --scene is required"); return 1;
    }
    if (args.robots.empty()) {
        PrintUsage(argv[0]); LOG_ERROR("main: at least one --robot is required"); return 1;
    }

    // ── 1. Renderer ───────────────────────────────────────────────────────
    Renderer renderer;
    renderer.Init(args.width, args.height, "FRC Sim 3D", args.target_fps);
    renderer.SetWireframe(args.wireframe);

    if (args.stream)
        renderer.EnableStreaming(args.stream_host, args.stream_port, args.stream_fps);

    std::string scene_display = args.scene;
    { auto s=scene_display.rfind('/'); if(s!=std::string::npos) scene_display=scene_display.substr(s+1);
      auto d=scene_display.rfind('.'); if(d!=std::string::npos) scene_display=scene_display.substr(0,d);
      for(auto &c:scene_display) c=(char)toupper(c); }

    LoadCtx lctx;
    lctx.scene_name = scene_display;

    // ── 2. Motor registry ─────────────────────────────────────────────────
    lctx.phase="LOADING MOTORS"; lctx.detail="assets/motors/"; lctx.overall=0.05f;
    DrawLoadingFrame(lctx);
    MotorRegistry motors;
    motors.LoadFromDirectory("assets/motors");

    // ── 3. Load scene ─────────────────────────────────────────────────────
    lctx.phase="PARSING SCENE"; lctx.detail=args.scene.c_str(); lctx.overall=0.10f;
    DrawLoadingFrame(lctx);
    SceneData scene = LoadScene(args.scene, motors);

    // ── 4. Physics world ──────────────────────────────────────────────────
    lctx.phase="INITIALISING PHYSICS"; lctx.detail="Jolt Physics"; lctx.overall=0.20f;
    DrawLoadingFrame(lctx);
    SimWorld world;
    world.Init();
    world.SetPhysicsDt(args.dt);

    // ── 5. Spawn non-robot scene bodies ───────────────────────────────────
    int total_bodies = (int)scene.bodies.size() + (int)args.robots.size();

    for (int i = 0; i < (int)scene.bodies.size(); ++i)
    {
        auto &req = scene.bodies[i];
        lctx.phase="LOADING MODELS"; lctx.detail=req.def.name.c_str();
        lctx.cur=i; lctx.total=total_bodies;
        lctx.overall=0.25f+0.40f*((float)i/total_bodies);
        DrawLoadingFrame(lctx);

        PreloadMesh(&req.def);
        auto id = world.SpawnBody(req.def, req.position.data(), req.orientation.data());
        if (id.IsInvalid())
            LOG_WARN("main: body '%s' failed to spawn", req.def.name.c_str());
    }

    // ── 6. Spawn robots + build mechanisms ───────────────────────────────
    // Store robot defs here so pointers remain valid for the sim lifetime
    std::vector<BodyDef>                          robot_defs;
    std::vector<std::unique_ptr<MechanismSystem>> all_mechanisms;
    std::vector<int>                              robot_motor_counts;

    robot_defs.reserve(args.robots.size());

    for (int ri = 0; ri < (int)args.robots.size(); ++ri)
    {
        auto &ra = args.robots[ri];

        lctx.phase="LOADING ROBOT";
        lctx.detail=ra.def_path.c_str();
        lctx.cur=(int)scene.bodies.size()+ri;
        lctx.total=total_bodies;
        lctx.overall=0.25f+0.40f*((float)lctx.cur/total_bodies);
        DrawLoadingFrame(lctx);

        auto maybe = LoadBodyDef(ra.def_path, motors);
        if (!maybe) {
            LOG_ERROR("main: failed to load robot def: %s", ra.def_path.c_str());
            robot_motor_counts.push_back(0);
            all_mechanisms.push_back(nullptr);
            continue;
        }
        robot_defs.push_back(std::move(*maybe));
        BodyDef &def = robot_defs.back();

        // Spawn position from scene robot_spawns[ri], fallback if not defined
        float pos[3] = {0.f, 0.051f, (float)ri * 1.5f};
        float rot[4] = {0.f, 0.f, 0.f, 1.f};
        if (ri < (int)scene.robot_spawns.size()) {
            auto &rs = scene.robot_spawns[ri];
            pos[0]=rs.position[0]; pos[1]=rs.position[1]; pos[2]=rs.position[2];
            rot[0]=rs.orientation[0]; rot[1]=rs.orientation[1];
            rot[2]=rs.orientation[2]; rot[3]=rs.orientation[3];
        }

        PreloadMesh(&def);
        auto id = world.SpawnBody(def, pos, rot);
        if (id.IsInvalid()) {
            LOG_ERROR("main: robot[%d] '%s' failed to spawn", ri, def.name.c_str());
            robot_motor_counts.push_back(0);
            all_mechanisms.push_back(nullptr);
            continue;
        }

        int body_idx = world.BodyCount() - 1;
        world.AddRobotIndex(body_idx);
        robot_motor_counts.push_back((int)def.motors.size());
        LOG_INFO("main: robot[%d] '%s' body_idx=%d motors=%d",
                 ri, def.name.c_str(), body_idx, (int)def.motors.size());

        // Mechanisms — from scene robot_spawns[ri] if defined
        if (ri < (int)scene.robot_spawns.size() && scene.robot_spawns[ri].has_mechanisms)
        {
            auto &rs = scene.robot_spawns[ri];
            all_mechanisms.push_back(std::make_unique<MechanismSystem>(
                world, rs.intake, rs.shooter, body_idx));
            LOG_INFO("main: robot[%d] mechanisms created  intake_cap=%d",
                     ri, rs.intake.max_capacity);
        }
        else
        {
            all_mechanisms.push_back(nullptr);
            LOG_INFO("main: robot[%d] no mechanisms", ri);
        }
    }

    lctx.cur=total_bodies; lctx.overall=0.80f;
    DrawLoadingFrame(lctx);

    // ── 7. Sim loop ───────────────────────────────────────────────────────
    lctx.phase="STARTING SIMULATION"; lctx.detail="physics thread"; lctx.overall=0.88f;
    DrawLoadingFrame(lctx);

    std::vector<MechanismSystem*> mech_ptrs;
    for (auto &m : all_mechanisms) mech_ptrs.push_back(m.get());

    ForceApplicator forces(world, motors, world.GetContactListener());
    SimLoop sim(world, &forces, std::move(mech_ptrs), args.dt, args.speed);

    sim.Start();

    // ── 8. NT clients — one per robot ─────────────────────────────────────
    std::vector<std::unique_ptr<NTClient>> nt_clients;
    const auto &robot_indices = world.GetRobotIndices();

    int spawn_slot = 0;
    for (int ri = 0; ri < (int)args.robots.size(); ++ri)
    {
        if (robot_motor_counts[ri] == 0) continue;  // failed spawn, no NT client
        if (spawn_slot >= (int)robot_indices.size()) break;

        auto &ra = args.robots[ri];
        lctx.phase="CONNECTING NT4";
        char addr[64]; snprintf(addr,sizeof(addr),"%s:%d [robot %d]",
                                ra.nt_host.c_str(), ra.nt_port, ri);
        lctx.detail=addr; lctx.overall=0.90f+0.08f*(float)ri/args.robots.size();
        DrawLoadingFrame(lctx);

        auto client = std::make_unique<NTClient>();
        client->Init(ra.nt_host, ra.nt_port, world,
                     robot_motor_counts[ri],
                     robot_indices[spawn_slot],      // ← spawn_slot
                     all_mechanisms[ri].get());
        ++spawn_slot;
        nt_clients.push_back(std::move(client));
        LOG_INFO("main: NT client[%d] → %s:%d", ri, ra.nt_host.c_str(), ra.nt_port);
    }

    lctx.phase="READY"; lctx.detail=""; lctx.cur=1; lctx.total=1; lctx.overall=1.0f;
    DrawLoadingFrame(lctx);


    renderer.SetWallTimeOffset(logger::elapsed());
 
    // ── 9. Main loop ──────────────────────────────────────────────────────
    while (!renderer.ShouldClose())
    {
        WorldSnapshot snapshot = sim.GetSnapshot();

        float frame_dt    = GetFrameTime();
        bool  any_connected = false;
        float best_ping   = -1.0f;

        for (auto &nt : nt_clients) {
            nt->Tick(snapshot, frame_dt);
            if (nt->IsConnected()) {
                any_connected = true;
                float p = nt->Ping();
                if (p >= 0) best_ping = p;
            }
        }

        renderer.DrawFrame(snapshot, any_connected,
                           sim.MeasuredHz(), sim.TargetHz(), best_ping);
    }

    // ── 10. Shutdown ──────────────────────────────────────────────────────
    sim.Stop();
    for (auto &nt : nt_clients) nt->Shutdown();
    renderer.Shutdown();
    LOG_INFO("main: clean exit");
    return 0;
}