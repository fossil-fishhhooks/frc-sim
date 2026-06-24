#pragma once
#include "core/Snapshot.h"
#include "render/MeshCache.h"
#include "io/Raycaster.h"

#include <sokol_gfx.h>
#include <sokol_app.h>

#include <memory>
#include <vector>

class Renderer {
public:
    void Init(int width, int height, const char* title, int target_fps);
    void Shutdown();
    bool ShouldClose() const;

    void DrawFrame(const WorldSnapshot& snapshot,
        bool nt_connected,
        float sim_hz, float target_hz, float nt_staleness_ms);

    bool m_cameraLocked = false;
    void SetWallTimeOffset(float ms) { m_wall_time_offset_ms = ms; }
    void SetFieldExtents(float hx, float hz) {
        m_field_half_extents[0] = hx;
        m_field_half_extents[1] = hz;
    }
    

    MeshCache& GetMeshCache() { return m_mesh_cache; }
    void SetRaycasters(std::vector<Raycaster*> rc) { m_raycasters = std::move(rc); }
    void HandleEvent(const sapp_event* e);

private:
    struct Camera {
        float pos[3] = { 5, 4, 5 };
        float target[3] = { 0, 0, 0 };
        float up[3] = { 0, 1, 0 };
        float fov = 60.0f;
        float yaw = -45.0f;
        float pitch = -30.0f;
        float dist = 7.5f;
    } m_cam;

    int m_target_fps = 0;
    uint64_t m_pace_stamp = 0;
    float m_wall_time_offset_ms = 0.0f;
    bool m_alt_pressed = false;

    bool m_mouse_down = false;
    float m_mouse_last_x = 0, m_mouse_last_y = 0;
    bool m_keys[256] = {};
    uint64_t m_stamp = 0;

    sg_pipeline m_pipeline = {};
    sg_shader m_shader = {};
    sg_pass_action m_pass_action = {};

    // Shadow mapping
    static constexpr int SHADOW_MAP_SIZE = 4096;
    float m_field_half_extents[2] = { 8.0f, 8.0f }; // set via SetFieldExtents()
    sg_image m_shadow_depth = {};
    sg_view m_shadow_depth_att_view = {};
    sg_view m_shadow_tex_view = {};
    sg_sampler m_shadow_sampler = {};
    sg_pipeline m_shadow_pipeline = {};
    sg_shader m_shadow_shader = {};

    MeshCache m_mesh_cache;

   

    std::vector<Raycaster*> m_raycasters;

    sg_image m_shadow_color = {};
    sg_view  m_shadow_color_att_view = {};

    void SetupCamera();
    void BuildProjMatrix(float out[16], float fov, float near, float far) const;
    void BuildViewMatrix(float out[16]) const;
    void UpdateCamera(float dt);

    void DrawForceVectors(const WorldSnapshot& snapshot);
    void DrawLightGizmos();
    void DrawRaycasts(const std::vector<Raycaster*>& rcs);

    void BuildMatrix(float out[16], const float pos[3], const float rot[4]) const;
    void BuildLightVPMatrix(float out[16]) const;
};