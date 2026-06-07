///// Written by AI + human edits
#include "render/Renderer.h"
#include "render/BodyDraw.h"
#include "render/DebugOverlay.h"
#include "io/EasyLog.h"
#include "raymath.h"
#include <rlgl.h>
#include <cmath>
#include <cstdio>
#include <cstring>

static bool FileExists_safe(const char *path) { return FileExists(path); }

// ── Light position (single overhead bar) ─────────────────────────────────────
// Adjust these to match your field scale.
static constexpr float LIGHT_X = 0.0f;
static constexpr float LIGHT_Y = 6.0f;
static constexpr float LIGHT_Z = 0.0f;

// ── Init ──────────────────────────────────────────────────────────────────────

void Renderer::Init(int width, int height, const char *title, int target_fps)
{
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    if (target_fps <= 0)
    {
        LOG_WARN("Renderer: uncapped FPS");
        ClearWindowState(FLAG_VSYNC_HINT);
    }
    InitWindow(width, height, title);
    SetTargetFPS(target_fps);

    m_camera.position = {5.0f, 4.0f, 5.0f};
    m_camera.target = {0.0f, 0.0f, 0.0f};
    m_camera.up = {0.0f, 1.0f, 0.0f};
    m_camera.fovy = 60.0f;
    m_camera.projection = CAMERA_PERSPECTIVE;

    // ── Load shader ───────────────────────────────────────────────────────
    const char *vs_path = "./assets/shader/lighting.vs";
    const char *fs_path = "./assets/shader/lighting.fs";

    if (FileExists_safe(vs_path) && FileExists_safe(fs_path))
    {
        m_shader = LoadShader(vs_path, fs_path);
        m_shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(m_shader, "matModel");

        m_locViewPos = GetShaderLocation(m_shader, "viewPos");
        m_locAmbient = GetShaderLocation(m_shader, "ambient");
        m_locLightSpaceMat = GetShaderLocation(m_shader, "lightSpaceMatrix");
        m_locShadowMap = GetShaderLocation(m_shader, "shadowMap");

        char buf[64];
        for (int i = 0; i < 2; ++i)
        {
            snprintf(buf, sizeof(buf), "lights[%d].enabled", i);
            m_lightLocs[i].enabled = GetShaderLocation(m_shader, buf);
            snprintf(buf, sizeof(buf), "lights[%d].type", i);
            m_lightLocs[i].type = GetShaderLocation(m_shader, buf);
            snprintf(buf, sizeof(buf), "lights[%d].position", i);
            m_lightLocs[i].position = GetShaderLocation(m_shader, buf);
            snprintf(buf, sizeof(buf), "lights[%d].target", i);
            m_lightLocs[i].target = GetShaderLocation(m_shader, buf);
            snprintf(buf, sizeof(buf), "lights[%d].color", i);
            m_lightLocs[i].color = GetShaderLocation(m_shader, buf);
        }

        SetupLights();
        m_shaderLoaded = true;

        // Shadow map only available if the new shader has the uniforms.
        // Old lighting.fs won't have shadowMap or lightSpaceMatrix — that's fine,
        // GetShaderLocation returns -1 and we just skip the shadow pass.
        if (m_locShadowMap >= 0 && m_locLightSpaceMat >= 0)
        {
            // ── Create shadow FBO + depth texture ─────────────────────────
            m_shadowFBO = rlLoadFramebuffer();
            rlEnableFramebuffer(m_shadowFBO);

            m_shadowDepthTex = rlLoadTextureDepth(SHADOW_MAP_SIZE, SHADOW_MAP_SIZE, false);
            rlFramebufferAttach(m_shadowFBO, m_shadowDepthTex,
                                RL_ATTACHMENT_DEPTH,
                                RL_ATTACHMENT_TEXTURE2D, 0);

            if (rlFramebufferComplete(m_shadowFBO))
            {
                m_shadowEnabled = true;
                LOG_INFO("Renderer: shadow map %dx%d ready", SHADOW_MAP_SIZE, SHADOW_MAP_SIZE);
            }
            else
            {
                LOG_WARN("Renderer: shadow FBO incomplete — shadows disabled");
                rlUnloadFramebuffer(m_shadowFBO);
                m_shadowFBO = 0;
            }

            rlDisableFramebuffer();

            // Bind shadow map to texture slot 1 (slot 0 = diffuse, owned by raylib)
            if (m_shadowEnabled)
            {
                BeginShaderMode(m_shader);
                int slot = 1;
                SetShaderValue(m_shader, m_locShadowMap, &slot, SHADER_UNIFORM_INT);
                EndShaderMode();
            }

            BuildLightSpaceMatrix(LIGHT_X, LIGHT_Y, LIGHT_Z);
        }
        else
        {
            LOG_INFO("Renderer: old shader detected: shadows not available");
        }

        LOG_INFO("Renderer: shader loaded  shadows=%s", m_shadowEnabled ? "yes" : "no");
    }
    else
    {
        LOG_WARN("Renderer: shader files not found — unlit fallback");
    }

    LOG_INFO("Renderer: window %dx%d '%s'", width, height, title);
}

// ── Light space matrix (orthographic, looking straight down from bar) ─────────

void Renderer::BuildLightSpaceMatrix(float lx, float ly, float lz)
{
    // Orthographic projection covering the whole field.
    // Adjust left/right/bottom/top to your field size in metres.
    // FRC 2024 field is ~16.5m × 8.2m — using 12×7 with headroom.
    Matrix lightProj = MatrixOrtho(-12.0f, 12.0f, -7.0f, 7.0f, 0.1f, 30.0f);

    // View: look from light position straight down to field centre
    Matrix lightView = MatrixLookAt(
        {lx, ly, lz},       // eye
        {0.0f, 0.0f, 0.0f}, // target
        {0.0f, 0.0f, 1.0f}  // up — Z since we're looking straight down Y
    );

    Matrix lsm = MatrixMultiply(lightView, lightProj);
    // Store column-major for glUniformMatrix4fv
    float *dst = m_lightSpaceMat;
    dst[0] = lsm.m0;
    dst[1] = lsm.m1;
    dst[2] = lsm.m2;
    dst[3] = lsm.m3;
    dst[4] = lsm.m4;
    dst[5] = lsm.m5;
    dst[6] = lsm.m6;
    dst[7] = lsm.m7;
    dst[8] = lsm.m8;
    dst[9] = lsm.m9;
    dst[10] = lsm.m10;
    dst[11] = lsm.m11;
    dst[12] = lsm.m12;
    dst[13] = lsm.m13;
    dst[14] = lsm.m14;
    dst[15] = lsm.m15;
}

// ── Lights ────────────────────────────────────────────────────────────────────

void Renderer::SetupLights()
{
    // Light 0: overhead point light (the bar)
    int one = 1, point = 1;
    SetShaderValue(m_shader, m_lightLocs[0].enabled, &one, SHADER_UNIFORM_INT);
    SetShaderValue(m_shader, m_lightLocs[0].type, &point, SHADER_UNIFORM_INT);
    float bar_pos[3] = {LIGHT_X, LIGHT_Y, LIGHT_Z};
    float bar_target[3] = {0.0f, 0.0f, 0.0f};
    float bar_color[4] = {1.8f, 1.7f, 1.62f, 1.0f}; // was 1.0 — HDR value, compensates for falloff
    SetShaderValue(m_shader, m_lightLocs[0].position, bar_pos, SHADER_UNIFORM_VEC3);
    SetShaderValue(m_shader, m_lightLocs[0].target, bar_target, SHADER_UNIFORM_VEC3);
    SetShaderValue(m_shader, m_lightLocs[0].color, bar_color, SHADER_UNIFORM_VEC4);

    // Light 1: disabled
    int zero = 0;
    SetShaderValue(m_shader, m_lightLocs[1].enabled, &zero, SHADER_UNIFORM_INT);

    float ambient[4] = {0.08f, 0.08f, 0.10f, 1.0f};
    SetShaderValue(m_shader, m_locAmbient, ambient, SHADER_UNIFORM_VEC4);
}

void Renderer::UpdateLightUniforms()
{
    float vp[3] = {m_camera.position.x, m_camera.position.y, m_camera.position.z};
    SetShaderValue(m_shader, m_locViewPos, vp, SHADER_UNIFORM_VEC3);

    if (m_shadowEnabled)
        SetShaderValueMatrix(m_shader, m_locLightSpaceMat,
                             *(Matrix *)m_lightSpaceMat);
}

// ── Shadow pass ───────────────────────────────────────────────────────────────
// Render every body into the depth texture from the light's POV.
// We use a minimal depth-only shader built into rlgl.

void Renderer::RenderShadowPass(const WorldSnapshot &snapshot)
{
    rlEnableFramebuffer(m_shadowFBO);
    rlViewport(0, 0, SHADOW_MAP_SIZE, SHADOW_MAP_SIZE);
    rlClearScreenBuffers();
    rlEnableDepthTest();

    // Build a temporary camera from light's POV for DrawBodySnapshot
    Camera3D lightCam{};
    lightCam.position = {LIGHT_X, LIGHT_Y, LIGHT_Z};
    lightCam.target = {0.0f, 0.0f, 0.0f};
    lightCam.up = {0.0f, 0.0f, 1.0f};
    lightCam.fovy = 90.0f;
    lightCam.projection = CAMERA_ORTHOGRAPHIC;

    BeginMode3D(lightCam);
    for (const auto &body : snapshot.bodies)
        DrawBodySnapshot(body, nullptr, false); // nullptr = no shader, depth only
    EndMode3D();

    rlDisableFramebuffer();
    rlViewport(0, 0, GetScreenWidth(), GetScreenHeight());
}

// ── Shutdown ──────────────────────────────────────────────────────────────────

void Renderer::Shutdown()
{
    m_stream.Shutdown();
    if (m_shadowEnabled)
    {
        rlUnloadFramebuffer(m_shadowFBO);
        rlUnloadTexture(m_shadowDepthTex);
    }
    if (m_shaderLoaded)
        UnloadShader(m_shader);
    UnloadAllMeshes();
    CloseWindow();
    LOG_INFO("Renderer: shutdown");
}

bool Renderer::ShouldClose() const { return WindowShouldClose(); }

// ── Main draw ─────────────────────────────────────────────────────────────────

void Renderer::DrawFrame(const WorldSnapshot &snapshot,
                         bool nt_connected,
                         float sim_hz, float target_hz, float nt_staleness_ms)
{
    if (IsKeyPressed(KEY_TAB))
        m_cameraLocked = !m_cameraLocked;
    if (m_cameraLocked)
        UpdateCamera(&m_camera, CAMERA_FREE);

    if (m_shaderLoaded)
        UpdateLightUniforms();

    // ── Shadow pass (depth only, into FBO) ───────────────────────────────
    if (m_shadowEnabled)
    {
        RenderShadowPass(snapshot);

        // Bind the depth texture to slot 1 so the main shader can sample it
        rlActiveTextureSlot(1);
        rlEnableTexture(m_shadowDepthTex);
    }

    // ── Main pass ─────────────────────────────────────────────────────────
    BeginDrawing();
    ClearBackground({28, 28, 32, 255});
    BeginMode3D(m_camera);

    for (const auto &body : snapshot.bodies)
        DrawBodySnapshot(body, m_shaderLoaded ? &m_shader : nullptr, m_wireframe);

    DrawLightGizmos();
    DrawForceVectors(snapshot);
    EndMode3D();

    if (m_shadowEnabled)
    {
        rlActiveTextureSlot(1);
        rlDisableTexture();
        rlActiveTextureSlot(0);
    }

    DrawDebugOverlay(snapshot, nt_connected, sim_hz, target_hz, nt_staleness_ms,m_wall_time_offset_ms);
    DrawText("WASD: movement  RMB drag: cam angle  EQ+Arrows: rotate view  Scroll: zoom   ESC: quit  TAB: lock/unlock camera",
             10, GetScreenHeight() - 20, 12, {200, 10, 10, 255});
    EndDrawing();
    if (m_stream.IsRunning())
    {
        Image frame = LoadImageFromScreen();
        m_stream.PushFrame(frame.data, frame.width, frame.height);
        UnloadImage(frame);
    }
}

void Renderer::EnableStreaming(const std::string &host, int port, int fps)
{
    m_stream_fps = fps;
    m_stream.Init(host, port, GetScreenWidth(), GetScreenHeight(), fps);
}

// ── Light gizmos ──────────────────────────────────────────────────────────────

void Renderer::DrawLightGizmos()
{
    if (!m_shaderLoaded)
        return;
    DrawSphere({LIGHT_X, LIGHT_Y, LIGHT_Z}, 0.12f, {255, 245, 200, 255});
}

// ── Force vectors ─────────────────────────────────────────────────────────────

void Renderer::DrawForceVectors(const WorldSnapshot &snapshot)
{
    for (int ri = 0; ri < (int)snapshot.robot_indices.size(); ++ri)
    {
        int robot_index = snapshot.robot_indices[ri];
        if (robot_index < 0 || robot_index >= (int)snapshot.bodies.size())
            continue;
        const BodySnapshot &robot = snapshot.bodies[robot_index];
        if (robot.motors.empty())
            continue;

        Vector3 robot_pos = {robot.pos[0], robot.pos[1], robot.pos[2]};
        Quaternion robot_rot = {robot.rot[0], robot.rot[1], robot.rot[2], robot.rot[3]};

        for (int i = 0; i < (int)robot.motors.size(); ++i)
        {
            const MotorSnapshot &motor = robot.motors[i];
            Vector3 local_pos = {motor.position[0], motor.position[1], motor.position[2]};
            Vector3 world_pos = Vector3Add(robot_pos, Vector3RotateByQuaternion(local_pos, robot_rot));

            if (motor.normal_force > 0.5f)
            {
                float len = motor.normal_force * 0.01f;
                Vector3 tip = Vector3Add(world_pos, {0, len, 0});
                DrawArrow3D(world_pos, tip, motor.normal_force > 10000.0f ? ORANGE : GREEN, 20.0f);
            }

            if (std::abs(motor.tractive_force) > 0.1f)
            {
                float len = std::abs(motor.tractive_force) * 0.001f;
                Vector3 local_dir = {motor.direction[0], motor.direction[1], motor.direction[2]};
                if (motor.tractive_force < 0)
                    local_dir = Vector3Negate(local_dir);
                Vector3 world_dir = Vector3RotateByQuaternion(local_dir, robot_rot);
                Vector3 tip = Vector3Add(world_pos, Vector3Scale(world_dir, len));
                Color col = std::abs(motor.tractive_force) > 1000.0f ? ORANGE
                            : motor.slipping                          ? RED
                                                                      : BLUE;
                DrawArrow3D(world_pos, tip, col, 0.02f);
            }
        }
    }
}
void Renderer::DrawArrow3D(Vector3 start, Vector3 end, Color color, float thickness)
{
    DrawLine3D(start, end, color);

    Vector3 dir = Vector3Normalize(Vector3Subtract(end, start));
    Vector3 arrowhead = Vector3Subtract(end, Vector3Scale(dir, 0.05f));
    Vector3 head_end = Vector3Add(arrowhead, Vector3Scale(dir, 0.03f));

    Vector3 perp1 = Vector3CrossProduct(dir, {0, 1, 0});
    if (Vector3Length(perp1) < 0.001f)
        perp1 = Vector3CrossProduct(dir, {1, 0, 0});
    perp1 = Vector3Normalize(perp1);
    Vector3 perp2 = Vector3CrossProduct(dir, perp1);

    float w = 0.015f;
    DrawLine3D(head_end, Vector3Add(arrowhead, Vector3Scale(perp1, w)), color);
    DrawLine3D(head_end, Vector3Add(arrowhead, Vector3Scale(perp1, -w)), color);
    DrawLine3D(head_end, Vector3Add(arrowhead, Vector3Scale(perp2, w)), color);
    DrawLine3D(head_end, Vector3Add(arrowhead, Vector3Scale(perp2, -w)), color);
}