///// Written by AI
#include "render/Renderer.h"
#include "render/BodyDraw.h"
#include "render/DebugOverlay.h"
#include "io/EasyLog.h"
#include "raymath.h"
#include <cmath>
#include <cstdio>

static bool FileExists_safe(const char *path)
{
    // raylib's FileExists() — just a wrapper but named clearly here
    return FileExists(path);
}

void Renderer::Init(int width, int height, const char *title, int target_fps)
{
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    if (target_fps <= 0)
    {
        LOG_WARN("Renderer: slamming hardware for render!");
        ClearWindowState(FLAG_VSYNC_HINT);
    }
    InitWindow(width, height, title);
    SetTargetFPS(target_fps);

    m_camera.position = {5.0f, 4.0f, 5.0f};
    m_camera.target = {0.0f, 0.0f, 0.0f};
    m_camera.up = {0.0f, 1.0f, 0.0f};
    m_camera.fovy = 60.0f;
    m_camera.projection = CAMERA_PERSPECTIVE;

    // Load shader only if both files exist
    const char *vs_path = "./assets/shader/lighting.vs";
    const char *fs_path = "./assets/shader/lighting.fs";

    if (FileExists_safe(vs_path) && FileExists_safe(fs_path))
    {
        m_shader = LoadShader(vs_path, fs_path);
        // Tell raylib to supply matModel so the vert shader gets world-space positions
        m_shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(m_shader, "matModel");

        m_locViewPos = GetShaderLocation(m_shader, "viewPos");
        m_locAmbient = GetShaderLocation(m_shader, "ambient");

        // Cache per-light uniform locations
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
        LOG_INFO("Renderer: Blinn-Phong shader loaded");
    }
    else
    {
        LOG_WARN("Renderer: shader files not found, rendering without lighting");
    }

    LOG_INFO("Renderer: window %dx%d '%s'", width, height, title);
}

void Renderer::SetupLights()
{
    // Light 0: overhead bar light — centered above field
    int one = 1;
    SetShaderValue(m_shader, m_lightLocs[0].enabled, &one, SHADER_UNIFORM_INT);
    int point = 1; // LIGHT_POINT
    SetShaderValue(m_shader, m_lightLocs[0].type, &point, SHADER_UNIFORM_INT);
    float bar_pos[3] = {0.0f, 6.0f, 0.0f}; // height depends on your world scale
    float bar_target[3] = {0.0f, 0.0f, 0.0f};
    float bar_color[4] = {1.0f, 0.97f, 0.92f, 1.0f}; // slightly warm white
    SetShaderValue(m_shader, m_lightLocs[0].position, bar_pos, SHADER_UNIFORM_VEC3);
    SetShaderValue(m_shader, m_lightLocs[0].target, bar_target, SHADER_UNIFORM_VEC3);
    SetShaderValue(m_shader, m_lightLocs[0].color, bar_color, SHADER_UNIFORM_VEC4);

    // Light 1: disabled
    int zero = 0;
    SetShaderValue(m_shader, m_lightLocs[1].enabled, &zero, SHADER_UNIFORM_INT);

    // Ambient: just enough to see undersides, not enough to wash shadows
    float ambient[4] = {0.08f, 0.08f, 0.10f, 1.0f};
    SetShaderValue(m_shader, m_locAmbient, ambient, SHADER_UNIFORM_VEC4);
}

void Renderer::UpdateLightUniforms()
{
    // Only viewPos changes per frame (camera moves); lights are static
    float vp[3] = {m_camera.position.x,
                   m_camera.position.y,
                   m_camera.position.z};
    SetShaderValue(m_shader, m_locViewPos, vp, SHADER_UNIFORM_VEC3);
}

void Renderer::Shutdown()
{
    if (m_shaderLoaded)
        UnloadShader(m_shader);
    UnloadAllMeshes();
    CloseWindow();
    LOG_INFO("Renderer: shutdown");
}

bool Renderer::ShouldClose() const { return WindowShouldClose(); }

void Renderer::DrawFrame(const WorldSnapshot &snapshot,
                         bool nt_connected,
                         float sim_hz, float target_hz)
{
    if (IsKeyPressed(KEY_TAB))
        m_cameraLocked = !m_cameraLocked;
    if (m_cameraLocked)
        UpdateCamera(&m_camera, CAMERA_FREE);

    if (m_shaderLoaded)
        UpdateLightUniforms();

    BeginDrawing();
    ClearBackground({28, 28, 32, 255});
    BeginMode3D(m_camera);

    for (const auto &body : snapshot.bodies)
        DrawBodySnapshot(body, m_shaderLoaded ? &m_shader : nullptr);

    DrawForceVectors(snapshot);
    EndMode3D();

    DrawDebugOverlay(snapshot, nt_connected, sim_hz, target_hz);
    DrawText("WASD: movement  RMB drag: cam angle  EQ+Arrows: rotate view  Scroll: zoom   ESC: quit  TAB: lock/unlock camera",
             10, GetScreenHeight() - 20, 12, {200, 10, 10, 255});
    EndDrawing();
}

// Draw force vectors as arrows from motor attachment points
void Renderer::DrawForceVectors(const WorldSnapshot& snapshot)
{
    if (snapshot.robot_index < 0 || snapshot.robot_index >= (int)snapshot.bodies.size())
        return;

    const BodySnapshot& robot = snapshot.bodies[snapshot.robot_index];
    if (robot.motors.empty())
        return;

    // Get robot world position and rotation
    Vector3 robot_pos = {robot.pos[0], robot.pos[1], robot.pos[2]};
    Quaternion robot_rot = {robot.rot[0], robot.rot[1], robot.rot[2], robot.rot[3]};

    for (int i = 0; i < (int)robot.motors.size(); ++i) {
        const MotorSnapshot& motor = robot.motors[i];

        // Motor attachment point in world space
        Vector3 local_pos = {motor.position[0], motor.position[1], motor.position[2]};
        Vector3 world_pos = Vector3Add(robot_pos, Vector3RotateByQuaternion(local_pos, robot_rot));

        // Normal force vector (up, green/red if excessive)
        if (motor.normal_force > 0.5f) {
            float arrow_length = motor.normal_force * 0.01f;  // Scale N to meters for display (10x bigger)
            Vector3 normal_end = Vector3Add(world_pos, {0, arrow_length, 0});
            Color normal_color = (motor.normal_force > 10000.0f) ? ORANGE : GREEN;  // Red if >1000N (crazy large)
            DrawArrow3D(world_pos, normal_end, normal_color, 20.0f);
        }

        // Tractive force vector (motor direction, blue/orange/red if excessive)
        if (std::abs(motor.tractive_force) > 0.1f) {
            float arrow_length = std::abs(motor.tractive_force) * 0.001f;  // Scale N to meters (10x bigger)
            Vector3 local_dir = {motor.direction[0], motor.direction[1], motor.direction[2]};            
            // Flip direction based on force sign (voltage direction)
            if (motor.tractive_force < 0) {
                local_dir = Vector3Negate(local_dir);
            }
                        Vector3 world_dir = Vector3RotateByQuaternion(local_dir, robot_rot);
            Vector3 traction_end = Vector3Add(world_pos, Vector3Scale(world_dir, arrow_length));

            Color traction_color;
            if (std::abs(motor.tractive_force) > 1000.0f) {  // Crazy large tractive force
                traction_color = ORANGE;
            } else if (motor.slipping) {
                traction_color = RED;
            } else {
                traction_color = BLUE;
            }
            DrawArrow3D(world_pos, traction_end, traction_color, 0.02f);
        }
    }
}

// Draw a 3D arrow from start to end
void Renderer::DrawArrow3D(Vector3 start, Vector3 end, Color color, float thickness)
{
    // Draw the main line
    DrawLine3D(start, end, color);

    // Calculate arrowhead direction and position
    Vector3 direction = Vector3Normalize(Vector3Subtract(end, start));
    Vector3 arrowhead_pos = Vector3Subtract(end, Vector3Scale(direction, 0.05f));  // Arrowhead starts 5cm back

    // Arrowhead as a cone (approximated with lines)
    float head_length = 0.03f;  // 3cm arrowhead
    Vector3 head_end = Vector3Add(arrowhead_pos, Vector3Scale(direction, head_length));

    // Draw arrowhead lines
    Vector3 perp1 = Vector3CrossProduct(direction, {0, 1, 0});
    if (Vector3Length(perp1) < 0.001f) perp1 = Vector3CrossProduct(direction, {1, 0, 0});
    perp1 = Vector3Normalize(perp1);

    Vector3 perp2 = Vector3CrossProduct(direction, perp1);

    float head_width = 0.015f;  // 1.5cm wide
    Vector3 head_left = Vector3Add(arrowhead_pos, Vector3Scale(perp1, head_width));
    Vector3 head_right = Vector3Add(arrowhead_pos, Vector3Scale(perp1, -head_width));
    Vector3 head_up = Vector3Add(arrowhead_pos, Vector3Scale(perp2, head_width));
    Vector3 head_down = Vector3Add(arrowhead_pos, Vector3Scale(perp2, -head_width));

    DrawLine3D(head_end, head_left, color);
    DrawLine3D(head_end, head_right, color);
    DrawLine3D(head_end, head_up, color);
    DrawLine3D(head_end, head_down, color);
}
