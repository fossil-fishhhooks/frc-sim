#pragma once
#include "core/Snapshot.h"
#include <raylib.h>

// Match rlights.h layout exactly so SetShaderValue offsets are correct
struct LightUniform
{
    int enabled;
    int type;
    float position[3];
    float target[3];
    float color[4];
};

class Renderer
{
public:
    void Init(int width, int height, const char *title, int target_fps);
    void Shutdown();
    bool ShouldClose() const;

    // Draw one frame. Reads snapshot, updates camera, calls BodyDraw + overlay.
    void DrawFrame(const WorldSnapshot &snapshot,
                   bool nt_connected,
                   float sim_hz, float target_hz);

    bool m_cameraLocked = false;

private:
    Camera3D m_camera{};
    Shader m_shader{};
    bool m_shaderLoaded = false;

    // Cached uniform locations
    int m_locViewPos = -1;
    int m_locAmbient = -1;

    // Light uniform locations (per-field, per-light)
    struct LightLocs
    {
        int enabled, type, position, target, color;
    };
    LightLocs m_lightLocs[2]; // two lights: one directional sun, one fill

    void SetupLights();
    void UpdateLightUniforms();
    void DrawForceVectors(const WorldSnapshot &);
    void DrawArrow3D(Vector3, Vector3, Color, float);
};

