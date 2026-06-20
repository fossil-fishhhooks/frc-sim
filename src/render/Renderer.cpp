#include "render/Renderer.h"
#include "render/BodyDraw.h"
#include "render/DebugOverlay.h"
#include "io/EasyLog.h"

#include <sokol_time.h>
#include <sokol_glue.h>
#include <cstdio>
#include <cmath>
#include <cstring>

static constexpr float LIGHT_X = 0.0f;
static constexpr float LIGHT_Y = 6.0f;
static constexpr float LIGHT_Z = 0.0f;

// ── Inline matrix/vector math (no raylib) ────────────────────────────────────

static void mat4_identity(float m[16]) {
    memset(m, 0, 16*sizeof(float));
    m[0]=m[5]=m[10]=m[15]=1.0f;
}

static void mat4_mul(const float a[16], const float b[16], float out[16]) {
    float t[16];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float sum = 0;
            for (int k = 0; k < 4; ++k)
                sum += a[k*4+i] * b[j*4+k];
            t[j*4+i] = sum;
        }
    memcpy(out, t, sizeof(t));
}

static void mat4_perspective(float fov_y, float aspect, float zn, float zf, float out[16]) {
    memset(out, 0, 16*sizeof(float));
    float f = 1.0f / tanf(fov_y * 0.5f);
    out[0] = f / aspect;
    out[5] = f;
    out[10] = (zf + zn) / (zn - zf);
    out[11] = -1.0f;
    out[14] = (2.0f * zf * zn) / (zn - zf);
}

static void mat4_look_at(const float eye[3], const float center[3], const float up[3], float out[16]) {
    float f[3], s[3], u[3];
    f[0] = center[0]-eye[0]; f[1] = center[1]-eye[1]; f[2] = center[2]-eye[2];
    float flen = sqrtf(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
    if (flen > 1e-8f) { f[0]/=flen; f[1]/=flen; f[2]/=flen; }

    s[0] = f[1]*up[2] - f[2]*up[1];
    s[1] = f[2]*up[0] - f[0]*up[2];
    s[2] = f[0]*up[1] - f[1]*up[0];
    float slen = sqrtf(s[0]*s[0]+s[1]*s[1]+s[2]*s[2]);
    if (slen > 1e-8f) { s[0]/=slen; s[1]/=slen; s[2]/=slen; }

    u[0] = s[1]*f[2] - s[2]*f[1];
    u[1] = s[2]*f[0] - s[0]*f[2];
    u[2] = s[0]*f[1] - s[1]*f[0];

    out[0]=s[0]; out[1]=u[0]; out[2]=-f[0]; out[3]=0;
    out[4]=s[1]; out[5]=u[1]; out[6]=-f[1]; out[7]=0;
    out[8]=s[2]; out[9]=u[2]; out[10]=-f[2]; out[11]=0;
    out[12]=-(s[0]*eye[0]+s[1]*eye[1]+s[2]*eye[2]);
    out[13]=-(u[0]*eye[0]+u[1]*eye[1]+u[2]*eye[2]);
    out[14]=f[0]*eye[0]+f[1]*eye[1]+f[2]*eye[2];
    out[15]=1;
}

static void vec3_sub(const float a[3], const float b[3], float out[3]) {
    out[0]=a[0]-b[0]; out[1]=a[1]-b[1]; out[2]=a[2]-b[2];
}
static void vec3_add(const float a[3], const float b[3], float out[3]) {
    out[0]=a[0]+b[0]; out[1]=a[1]+b[1]; out[2]=a[2]+b[2];
}
static void vec3_scale(const float a[3], float s, float out[3]) {
    out[0]=a[0]*s; out[1]=a[1]*s; out[2]=a[2]*s;
}
static float vec3_dot(const float a[3], const float b[3]) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
static void vec3_cross(const float a[3], const float b[3], float out[3]) {
    out[0]=a[1]*b[2]-a[2]*b[1];
    out[1]=a[2]*b[0]-a[0]*b[2];
    out[2]=a[0]*b[1]-a[1]*b[0];
}
static void vec3_normalize(const float a[3], float out[3]) {
    float len = sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (len>1e-8f) { out[0]=a[0]/len; out[1]=a[1]/len; out[2]=a[2]/len; }
    else memcpy(out, a, 3*sizeof(float));
}

// ── Vertex & uniform layouts ────────────────────────────────────────────────

struct Vertex {
    float x, y, z;
    float nx, ny, nz;
};

struct VsUniforms {
    float model[16];
    float view[16];
    float projection[16];
};

struct FsUniforms {
    float color[4];
    float light_pos[4];
    float ambient[4];
    float view_pos[4];
};

// ── GLSL shaders (GL 3.3 core) ──────────────────────────────────────────────

static const char* vs_src = R"(
    #version 330
    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;
    layout(location = 0) in vec3 position;
    layout(location = 1) in vec3 normal;
    out vec3 v_normal;
    out vec3 v_pos;
    void main() {
        vec4 world_pos = model * vec4(position, 1.0);
        gl_Position = projection * view * world_pos;
        v_normal = mat3(model) * normal;
        v_pos = world_pos.xyz;
    }
)";

static const char* fs_src = R"(
    #version 330
    uniform vec4 color;
    uniform vec4 light_pos;
    uniform vec4 ambient;
    uniform vec4 view_pos;
    in vec3 v_normal;
    in vec3 v_pos;
    out vec4 frag_color;
    void main() {
        vec3 N = normalize(v_normal);
        vec3 L = normalize(light_pos.xyz - v_pos);
        float diff = max(dot(N, L), 0.0);
        vec3 amb = ambient.xyz * color.xyz;
        vec3 dif = diff * color.xyz;
        frag_color = vec4(amb + dif, color.a);
    }
)";

// ── Init ──────────────────────────────────────────────────────────────────────

void Renderer::Init(int width, int height, const char* title, int target_fps) {
    (void)width; (void)height; (void)title; (void)target_fps;
    // sokol_gfx is already initialized by sokol_app before init callback

    m_pass_action = {};
    m_pass_action.colors[0].load_action = SG_LOADACTION_CLEAR;
    m_pass_action.colors[0].clear_value = {0.11f, 0.11f, 0.125f, 1.0f};
    m_pass_action.depth.load_action = SG_LOADACTION_CLEAR;
    m_pass_action.depth.clear_value = 1.0f;

    // ── Shader ───────────────────────────────────────────────────────────
    sg_shader_desc shd = {};
    shd.vertex_func.source = vs_src;
    shd.vertex_func.entry = "main";
    shd.fragment_func.source = fs_src;
    shd.fragment_func.entry = "main";

    shd.uniform_blocks[0].stage = SG_SHADERSTAGE_VERTEX;
    shd.uniform_blocks[0].size = sizeof(VsUniforms);
    shd.uniform_blocks[0].layout = SG_UNIFORMLAYOUT_NATIVE;
    shd.uniform_blocks[0].glsl_uniforms[0] = {SG_UNIFORMTYPE_MAT4, 1, "model"};
    shd.uniform_blocks[0].glsl_uniforms[1] = {SG_UNIFORMTYPE_MAT4, 1, "view"};
    shd.uniform_blocks[0].glsl_uniforms[2] = {SG_UNIFORMTYPE_MAT4, 1, "projection"};

    shd.uniform_blocks[1].stage = SG_SHADERSTAGE_FRAGMENT;
    shd.uniform_blocks[1].size = sizeof(FsUniforms);
    shd.uniform_blocks[1].layout = SG_UNIFORMLAYOUT_NATIVE;
    shd.uniform_blocks[1].glsl_uniforms[0] = {SG_UNIFORMTYPE_FLOAT4, 1, "color"};
    shd.uniform_blocks[1].glsl_uniforms[1] = {SG_UNIFORMTYPE_FLOAT4, 1, "light_pos"};
    shd.uniform_blocks[1].glsl_uniforms[2] = {SG_UNIFORMTYPE_FLOAT4, 1, "ambient"};
    shd.uniform_blocks[1].glsl_uniforms[3] = {SG_UNIFORMTYPE_FLOAT4, 1, "view_pos"};

    m_shader = sg_make_shader(&shd);
    if (!m_shader.id) {
        LOG_ERROR("Renderer: failed to create shader");
    }

    // ── Solid pipeline ───────────────────────────────────────────────────
    sg_pipeline_desc pip = {};
    pip.layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3;
    pip.layout.attrs[0].buffer_index = 0;
    pip.layout.attrs[1].format = SG_VERTEXFORMAT_FLOAT3;
    pip.layout.attrs[1].buffer_index = 0;
    pip.layout.buffers[0].stride = sizeof(Vertex);
    pip.shader = m_shader;
    pip.index_type = SG_INDEXTYPE_UINT32;
    pip.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
    pip.depth.write_enabled = true;
    pip.cull_mode = SG_CULLMODE_BACK;
    m_pipeline = sg_make_pipeline(&pip);

    // ── Camera ───────────────────────────────────────────────────────────
    SetupCamera();
    m_stamp = stm_now();

    LOG_INFO("Renderer: sokol_gfx initialized");
}

void Renderer::Shutdown() {
    if (m_stream) m_stream->Shutdown();
    if (m_pipeline.id) sg_destroy_pipeline(m_pipeline);
    if (m_shader.id) sg_destroy_shader(m_shader);
    m_mesh_cache.UnloadAll();
    LOG_INFO("Renderer: shutdown");
}

bool Renderer::ShouldClose() const {
    return false;
}

void Renderer::EnableStreaming(int port, int fps) {
    m_stream_fps = fps;
    m_stream = std::make_unique<StreamEncoder>();
    m_stream->Init(port, sapp_width(), sapp_height(), fps);
}

// ── Camera ────────────────────────────────────────────────────────────────────

void Renderer::SetupCamera() {
    float sp = m_cam.dist * sinf(m_cam.pitch * 3.14159265f / 180.0f);
    float c = m_cam.dist * cosf(m_cam.pitch * 3.14159265f / 180.0f);
    m_cam.pos[0] = m_cam.target[0] + c * sinf(m_cam.yaw * 3.14159265f / 180.0f);
    m_cam.pos[1] = m_cam.target[1] - sp;
    m_cam.pos[2] = m_cam.target[2] + c * cosf(m_cam.yaw * 3.14159265f / 180.0f);
}

void Renderer::BuildProjMatrix(float out[16], float fov, float near, float far) const {
    float aspect = (float)sapp_width() / fmaxf(1.0f, (float)sapp_height());
    mat4_perspective(fov * 3.14159265f / 180.0f, aspect, near, far, out);
}

void Renderer::BuildViewMatrix(float out[16]) const {
    mat4_look_at(m_cam.pos, m_cam.target, m_cam.up, out);
}

void Renderer::UpdateCamera(float dt) {
    float speed = 5.0f * dt;
    float fwd[3], right[3];
    vec3_sub(m_cam.target, m_cam.pos, fwd);
    fwd[1] = 0;
    vec3_normalize(fwd, fwd);
    vec3_cross(fwd, m_cam.up, right);
    vec3_normalize(right, right);

    float move[3] = {0,0,0};
    if (m_keys[SAPP_KEYCODE_W]) vec3_add(move, fwd, move);
    if (m_keys[SAPP_KEYCODE_S]) vec3_sub(move, fwd, move);
    if (m_keys[SAPP_KEYCODE_A]) vec3_sub(move, right, move);
    if (m_keys[SAPP_KEYCODE_D]) vec3_add(move, right, move);
    if (m_keys[SAPP_KEYCODE_Q]) move[1] += 1;
    if (m_keys[SAPP_KEYCODE_E]) move[1] -= 1;
    vec3_scale(move, speed, move);
    vec3_add(m_cam.target, move, m_cam.target);
    vec3_add(m_cam.pos, move, m_cam.pos);
}

// ── Event handling ────────────────────────────────────────────────────────────

void Renderer::HandleEvent(const sapp_event* e) {
    switch (e->type) {
        case SAPP_EVENTTYPE_KEY_DOWN:
            if (e->key_code < 256) m_keys[e->key_code] = true;
            if (e->key_code == SAPP_KEYCODE_TAB) m_cameraLocked = !m_cameraLocked;
            if (e->key_code == SAPP_KEYCODE_LEFT_ALT || e->key_code == SAPP_KEYCODE_RIGHT_ALT)
                m_alt_pressed = true;
            break;
        case SAPP_EVENTTYPE_KEY_UP:
            if (e->key_code < 256) m_keys[e->key_code] = false;
            if (e->key_code == SAPP_KEYCODE_LEFT_ALT || e->key_code == SAPP_KEYCODE_RIGHT_ALT)
                m_alt_pressed = false;
            break;
        case SAPP_EVENTTYPE_MOUSE_DOWN:
            if (e->mouse_button == SAPP_MOUSEBUTTON_RIGHT) {
                m_mouse_down = true;
                m_mouse_last_x = e->mouse_x;
                m_mouse_last_y = e->mouse_y;
            }
            break;
        case SAPP_EVENTTYPE_MOUSE_UP:
            if (e->mouse_button == SAPP_MOUSEBUTTON_RIGHT) m_mouse_down = false;
            break;
        case SAPP_EVENTTYPE_MOUSE_MOVE:
            if (m_mouse_down && !m_cameraLocked) {
                float dx = e->mouse_x - m_mouse_last_x;
                float dy = e->mouse_y - m_mouse_last_y;
                m_mouse_last_x = e->mouse_x;
                m_mouse_last_y = e->mouse_y;
                m_cam.yaw += dx * 0.2f;
                m_cam.pitch += dy * 0.2f;
                m_cam.pitch = fmaxf(-89.0f, fminf(89.0f, m_cam.pitch));
            }
            break;
        case SAPP_EVENTTYPE_MOUSE_SCROLL:
            m_cam.dist *= (e->scroll_y > 0) ? 0.9f : 1.1f;
            if (m_cam.dist < 0.5f) m_cam.dist = 0.5f;
            if (m_cam.dist > 50.0f) m_cam.dist = 50.0f;
            break;
        default:
            break;
    }
}

// ── Draw helpers ──────────────────────────────────────────────────────────────

void Renderer::BuildMatrix(float out[16], const float pos[3], const float rot[4]) const {
    float r[16];
    QuatToMatrix(rot, r);
    r[12] = pos[0]; r[13] = pos[1]; r[14] = pos[2];
    memcpy(out, r, 16 * sizeof(float));
}

void Renderer::DrawLightGizmos() {
    (void)0;
}

void Renderer::DrawForceVectors(const WorldSnapshot& snap) {
    (void)snap;
}

void Renderer::DrawRaycasts(const std::vector<Raycaster*>& rcs) {
    (void)rcs;
}

// ── Main draw ─────────────────────────────────────────────────────────────────

void Renderer::DrawFrame(const WorldSnapshot& snapshot,
                          bool nt_connected,
                          float sim_hz, float target_hz, float nt_ping_ms) {
    float dt = stm_sec(stm_laptime(&m_stamp));
    if (dt > 0.1f) dt = 0.1f;

    if (!m_cameraLocked)
        UpdateCamera(dt);
    SetupCamera();

    // Per-frame matrices
    float proj[16], view[16];
    BuildProjMatrix(proj, m_cam.fov, 0.1f, 100.0f);
    BuildViewMatrix(view);

    // Per-frame FS uniforms (light, ambient, view_pos — color set per-body)
    FsUniforms fs_ub = {};
    fs_ub.light_pos[0] = LIGHT_X;
    fs_ub.light_pos[1] = LIGHT_Y;
    fs_ub.light_pos[2] = LIGHT_Z;
    fs_ub.light_pos[3] = 1.0f;
    fs_ub.ambient[0] = 0.08f;
    fs_ub.ambient[1] = 0.08f;
    fs_ub.ambient[2] = 0.10f;
    fs_ub.ambient[3] = 1.0f;
    fs_ub.view_pos[0] = m_cam.pos[0];
    fs_ub.view_pos[1] = m_cam.pos[1];
    fs_ub.view_pos[2] = m_cam.pos[2];
    fs_ub.view_pos[3] = 1.0f;

    // ── Begin render pass ────────────────────────────────────────────────
    sg_pass pass = { .action = m_pass_action, .swapchain = sglue_swapchain() };
    sg_begin_pass(&pass);

    // ── Solid pass ────────────────────────────────────────────────────────
    sg_apply_pipeline(m_pipeline);
    for (const auto& body : snapshot.bodies) {
        float model[16];
        BuildMatrix(model, body.pos, body.rot);
        VsUniforms vs_ub;
        memcpy(vs_ub.model, model, sizeof(model));
        memcpy(vs_ub.view, view, sizeof(view));
        memcpy(vs_ub.projection, proj, sizeof(proj));
        sg_apply_uniforms(0, SG_RANGE(vs_ub));

        float col[4];
        BodyColor(body.def, col);
        memcpy(fs_ub.color, col, sizeof(col));
        sg_apply_uniforms(1, SG_RANGE(fs_ub));

        DrawBodySnapshot(body, &m_mesh_cache, false);
    }

    // ── Debug overlay ─────────────────────────────────────────────────────
    DrawDebugOverlay(snapshot, nt_connected, sim_hz, target_hz, nt_ping_ms, m_wall_time_offset_ms);

    // ── End pass ──────────────────────────────────────────────────────────
    sg_end_pass();
    sg_commit();

    // ── Stream (dummy) ───────────────────────────────────────────────────
    if (m_stream) {
        m_stream_accum += dt;
        if (m_stream_accum >= 1.0f / m_stream_fps) {
            m_stream_accum -= 1.0f / m_stream_fps;
            m_stream->PushFrame(nullptr, sapp_width(), sapp_height());
        }
    }
}
