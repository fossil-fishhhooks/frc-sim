#pragma once
#include "core/Snapshot.h"
#include <raylib.h>
#include <rlgl.h>

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

    void DrawFrame(const WorldSnapshot &snapshot,
                   bool nt_connected,
                   float sim_hz, float target_hz);

    bool m_cameraLocked = false;

    void SetWireframe(bool enabled) { m_wireframe = enabled; }

private:
    Camera3D m_camera{};

    bool m_wireframe = false;

    // ── Main lighting shader ──────────────────────────────────────────────
    Shader m_shader{};
    bool m_shaderLoaded = false;
    bool m_shadowEnabled = false; // true only if shader has shadowMap uniform

    int m_locViewPos = -1;
    int m_locAmbient = -1;
    int m_locLightSpaceMat = -1; // uniform mat4 lightSpaceMatrix
    int m_locShadowMap = -1;     // uniform sampler2D shadowMap

    struct LightLocs
    {
        int enabled, type, position, target, color;
    };
    LightLocs m_lightLocs[2];

    // ── Shadow map resources ──────────────────────────────────────────────
    static constexpr int SHADOW_MAP_SIZE = 2048;
    unsigned int m_shadowFBO = 0;      // framebuffer
    unsigned int m_shadowDepthTex = 0; // depth texture bound to FBO

    // Light view+projection matrix — orthographic from overhead bar position
    // Recomputed once in SetupLights (static light, no need to update per frame)
    float m_lightSpaceMat[16]{};

    // ── Helpers ───────────────────────────────────────────────────────────
    void SetupLights();
    void UpdateLightUniforms();
    void RenderShadowPass(const WorldSnapshot &snapshot);
    void BuildLightSpaceMatrix(float light_x, float light_y, float light_z);

    void DrawLightGizmos();
    void DrawForceVectors(const WorldSnapshot &);
    void DrawArrow3D(Vector3, Vector3, Color, float);
};