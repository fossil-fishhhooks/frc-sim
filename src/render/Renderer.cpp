#include "render/Renderer.h"
#include "render/BodyDraw.h"
#include "render/DebugOverlay.h"
#include "io/EasyLog.h"

#include <sokol_time.h>
#include <sokol_glue.h>
#include <cstdio>
#include <cmath>
#include <cstring>

// ── File loader ──────────────────────────────────────────────────────────────

static bool ReadFile(const char* path, std::string& out) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    out.resize(len > 0 ? (size_t)len : 0);
    if (len > 0) fread(&out[0], 1, (size_t)len, f);
    fclose(f);
    return true;
}

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

static void mat4_ortho(float left, float right, float bottom, float top, float zn, float zf, float out[16]) {
    memset(out, 0, 16*sizeof(float));
    out[0]  = 2.0f / (right - left);
    out[5]  = 2.0f / (top - bottom);
    out[10] = -2.0f / (zf - zn);
    out[12] = -(right + left) / (right - left);
    out[13] = -(top + bottom) / (top - bottom);
    out[14] = -(zf + zn) / (zf - zn);
    out[15] = 1.0f;
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
    uint8_t r, g, b, a;
};

struct VsUniforms {
    float model[16];
    float view[16];
    float projection[16];
};

struct FsUniforms {
    float model_color[4];
    float ambient[4];
    float light_pos[4];
    float view_pos[4];
    float light_power;
    float light_vp[16];
};

struct ShadowVsUniforms {
    float light_vp[16];
    float model[16];
};

// Verify uniform struct sizes match GLSL layout
static_assert(sizeof(VsUniforms) == 192, "VsUniforms size mismatch (3 mat4)");
static_assert(sizeof(FsUniforms) == 132, "FsUniforms size mismatch");
static_assert(sizeof(ShadowVsUniforms) == 128, "ShadowVsUniforms size mismatch (2 mat4)");

// ── GLSL shader sources (fallback inline) ────────────────────────────────────

static const char* vs_src_main_fallback =
"#version 330\n"
"uniform mat4 model;\n"
"uniform mat4 view;\n"
"uniform mat4 projection;\n"
"layout(location = 0) in vec3 position;\n"
"layout(location = 1) in vec3 normal;\n"
"layout(location = 2) in vec4 color;\n"
"out vec3 v_normal;\n"
"out vec3 v_pos;\n"
"out vec4 v_color;\n"
"void main() {\n"
"    vec4 world_pos = model * vec4(position, 1.0);\n"
"    gl_Position = projection * view * world_pos;\n"
"    v_normal = mat3(model) * normal;\n"
"    v_pos = world_pos.xyz;\n"
"    v_color = color;\n"
"}\n";

static const char* fs_src_main_fallback =
"#version 330\n"
"uniform vec4 model_color;\n"
"uniform vec4 ambient;\n"
"uniform vec4 light_pos;\n"
"uniform vec4 view_pos;\n"
"uniform float light_power;\n"
"in vec3 v_normal;\n"
"in vec3 v_pos;\n"
"in vec4 v_color;\n"
"out vec4 frag_color;\n"
"void main() {\n"
"    vec3 N = normalize(v_normal);\n"
"    vec3 Lv = light_pos.xyz - v_pos;\n"
"    float dist = length(Lv);\n"
"    vec3 L = Lv / dist;\n"
"    float atten = 1.0 / (1.0 + 0.007 * dist * dist);\n"
"    float diff = max(dot(N, L), 0.0);\n"
"    vec3 base = model_color.rgb * v_color.rgb;\n"
"    vec3 amb = ambient.rgb * base;\n"
"    vec3 diffuse = diff * atten * base * light_power;\n"
"    vec3 V = normalize(view_pos.xyz - v_pos);\n"
"    vec3 H = normalize(L + V);\n"
"    float spec = pow(max(dot(N, H), 0.0), 32.0) * atten * light_power;\n"
"    frag_color = vec4(amb + diffuse + vec3(spec * 0.3), 1.0);\n"
"}\n";

static const char* vs_src_shadow_fallback =
"#version 330\n"
"uniform mat4 light_vp;\n"
"uniform mat4 model;\n"
"layout(location = 0) in vec3 position;\n"
"void main() {\n"
"    gl_Position = light_vp * model * vec4(position, 1.0);\n"
"}\n";

static const char* fs_src_shadow_fallback =
"#version 330\n"
"out vec4 frag_color;\n"
"void main() {\n"
"    float d = gl_FragCoord.z;\n"
"    frag_color = vec4(d, d, d, 1.0);\n"
"}\n";

// ── Init ──────────────────────────────────────────────────────────────────────

void Renderer::Init(int width, int height, const char* title, int target_fps) {
    (void)width; (void)height; (void)title; (void)target_fps;

    m_pass_action = {};
    m_pass_action.colors[0].load_action = SG_LOADACTION_CLEAR;
    m_pass_action.colors[0].clear_value = {0.11f, 0.11f, 0.125f, 1.0f};
    m_pass_action.depth.load_action = SG_LOADACTION_CLEAR;
    m_pass_action.depth.clear_value = 1.0f;

    // ── Load shader sources from files (fallback to inline) ───────────────
    std::string vs_main_str, fs_main_str, vs_shadow_str, fs_shadow_str;

    bool loaded = ReadFile("assets/shader/main.vs", vs_main_str);
    const char* vs_main = loaded ? vs_main_str.c_str() : vs_src_main_fallback;
    if (loaded) LOG_INFO("Renderer: loaded assets/shader/main.vs");

    loaded = ReadFile("assets/shader/main.fs", fs_main_str);
    const char* fs_main = loaded ? fs_main_str.c_str() : fs_src_main_fallback;
    if (loaded) LOG_INFO("Renderer: loaded assets/shader/main.fs");

    loaded = ReadFile("assets/shader/shadow.vs", vs_shadow_str);
    const char* vs_shadow = loaded ? vs_shadow_str.c_str() : vs_src_shadow_fallback;
    if (loaded) LOG_INFO("Renderer: loaded assets/shader/shadow.vs");

    loaded = ReadFile("assets/shader/shadow.fs", fs_shadow_str);
    const char* fs_shadow = loaded ? fs_shadow_str.c_str() : fs_src_shadow_fallback;
    if (loaded) LOG_INFO("Renderer: loaded assets/shader/shadow.fs");

    // ── Main shader ───────────────────────────────────────────────────────
    sg_shader_desc shd = {};
    shd.vertex_func.source = vs_main;
    shd.vertex_func.entry = "main";
    shd.fragment_func.source = fs_main;
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
    shd.uniform_blocks[1].glsl_uniforms[0] = {SG_UNIFORMTYPE_FLOAT4, 1, "model_color"};
    shd.uniform_blocks[1].glsl_uniforms[1] = {SG_UNIFORMTYPE_FLOAT4, 1, "ambient"};
    shd.uniform_blocks[1].glsl_uniforms[2] = {SG_UNIFORMTYPE_FLOAT4, 1, "light_pos"};
    shd.uniform_blocks[1].glsl_uniforms[3] = {SG_UNIFORMTYPE_FLOAT4, 1, "view_pos"};
    shd.uniform_blocks[1].glsl_uniforms[4] = {SG_UNIFORMTYPE_FLOAT, 1, "light_power"};
    shd.uniform_blocks[1].glsl_uniforms[5] = {SG_UNIFORMTYPE_MAT4, 1, "light_vp"};

    shd.views[0].texture.stage = SG_SHADERSTAGE_FRAGMENT;
    shd.views[0].texture.image_type = SG_IMAGETYPE_2D;

    shd.samplers[0].stage = SG_SHADERSTAGE_FRAGMENT;
    shd.samplers[0].sampler_type = SG_SAMPLERTYPE_FILTERING;

    shd.texture_sampler_pairs[0].stage = SG_SHADERSTAGE_FRAGMENT;
    shd.texture_sampler_pairs[0].view_slot = 0;
    shd.texture_sampler_pairs[0].sampler_slot = 0;
    shd.texture_sampler_pairs[0].glsl_name = "shadow_map";

    m_shader = sg_make_shader(&shd);
    if (!m_shader.id) {
        LOG_ERROR("Renderer: failed to create main shader");
    }

    // ── Solid pipeline ───────────────────────────────────────────────────
    sg_pipeline_desc pip = {};
    pip.color_count = 1;
    pip.colors[0].pixel_format = SG_PIXELFORMAT_RGBA8;
    pip.depth.pixel_format = SG_PIXELFORMAT_DEPTH_STENCIL;
    pip.layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3;
    pip.layout.attrs[0].buffer_index = 0;
    pip.layout.attrs[0].offset = 0;
    pip.layout.attrs[1].format = SG_VERTEXFORMAT_FLOAT3;
    pip.layout.attrs[1].buffer_index = 0;
    pip.layout.attrs[1].offset = 12;
    pip.layout.attrs[2].format = SG_VERTEXFORMAT_UBYTE4N;
    pip.layout.attrs[2].buffer_index = 0;
    pip.layout.attrs[2].offset = 24;
    pip.layout.buffers[0].stride = sizeof(Vertex);
    pip.shader = m_shader;
    pip.index_type = SG_INDEXTYPE_UINT32;
    pip.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
    pip.depth.write_enabled = true;
    pip.cull_mode = SG_CULLMODE_BACK;
    pip.face_winding = SG_FACEWINDING_CCW;
    m_pipeline = sg_make_pipeline(&pip);
    if (!m_pipeline.id) {
        LOG_ERROR("Renderer: failed to create main pipeline");
    }

    // ── Shadow shader ─────────────────────────────────────────────────────
    sg_shader_desc shd_shadow = {};
    shd_shadow.vertex_func.source = vs_shadow;
    shd_shadow.vertex_func.entry = "main";
    shd_shadow.fragment_func.source = fs_shadow;
    shd_shadow.fragment_func.entry = "main";

    shd_shadow.uniform_blocks[0].stage = SG_SHADERSTAGE_VERTEX;
    shd_shadow.uniform_blocks[0].size = sizeof(ShadowVsUniforms);
    shd_shadow.uniform_blocks[0].layout = SG_UNIFORMLAYOUT_NATIVE;
    shd_shadow.uniform_blocks[0].glsl_uniforms[0] = {SG_UNIFORMTYPE_MAT4, 1, "light_vp"};
    shd_shadow.uniform_blocks[0].glsl_uniforms[1] = {SG_UNIFORMTYPE_MAT4, 1, "model"};

    m_shadow_shader = sg_make_shader(&shd_shadow);
    if (!m_shadow_shader.id) {
        LOG_ERROR("Renderer: failed to create shadow shader");
    }

    // ── Shadow pipeline ───────────────────────────────────────────────────
    sg_pipeline_desc spip = {};
    spip.color_count = 1;
    spip.colors[0].pixel_format = SG_PIXELFORMAT_R32F;
    spip.depth.pixel_format = SG_PIXELFORMAT_DEPTH_STENCIL;
    spip.layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3;
    spip.layout.attrs[0].buffer_index = 0;
    spip.layout.attrs[0].offset = 0;
    spip.layout.buffers[0].stride = sizeof(Vertex);
    spip.shader = m_shadow_shader;
    spip.index_type = SG_INDEXTYPE_UINT32;
    spip.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
    spip.depth.write_enabled = true;
    spip.cull_mode = SG_CULLMODE_NONE;
    m_shadow_pipeline = sg_make_pipeline(&spip);

    // ── Shadow map images ─────────────────────────────────────────────────
    sg_image_desc depth_desc = {};
    depth_desc.width = SHADOW_MAP_SIZE;
    depth_desc.height = SHADOW_MAP_SIZE;
    depth_desc.pixel_format = SG_PIXELFORMAT_DEPTH_STENCIL;
    depth_desc.usage.depth_stencil_attachment = true;
    depth_desc.usage.immutable = true;
    m_shadow_depth = sg_make_image(&depth_desc);

    sg_image_desc color_desc = {};
    color_desc.width = SHADOW_MAP_SIZE;
    color_desc.height = SHADOW_MAP_SIZE;
    color_desc.pixel_format = SG_PIXELFORMAT_R32F;
    color_desc.usage.color_attachment = true;
    color_desc.usage.immutable = true;
    m_shadow_color = sg_make_image(&color_desc);

    // ── Shadow pass attachment views ──────────────────────────────────────
    sg_view_desc cav = {};
    cav.color_attachment.image = m_shadow_color;
    m_shadow_color_att_view = sg_make_view(&cav);

    sg_view_desc dav = {};
    dav.depth_stencil_attachment.image = m_shadow_depth;
    m_shadow_depth_att_view = sg_make_view(&dav);

    // ── Shadow map texture view (for sampling in main pass) ──────────────
    sg_view_desc tv = {};
    tv.texture.image = m_shadow_depth;
    m_shadow_tex_view = sg_make_view(&tv);

    sg_view_desc color_tv = {};
    color_tv.texture.image = m_shadow_color;
    m_shadow_color_tex_view = sg_make_view(&color_tv);

    // ── Shadow sampler ────────────────────────────────────────────────────
    sg_sampler_desc smp_desc = {};
    smp_desc.min_filter = SG_FILTER_NEAREST;
    smp_desc.mag_filter = SG_FILTER_NEAREST;
    smp_desc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
    smp_desc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
    m_shadow_sampler = sg_make_sampler(&smp_desc);

    // ── Camera ───────────────────────────────────────────────────────────
    SetupCamera();
    m_stamp = stm_now();

    LOG_INFO("Renderer: sokol_gfx initialized");
}

void Renderer::Shutdown() {
    if (m_stream) m_stream->Shutdown();
    if (m_shadow_sampler.id) sg_destroy_sampler(m_shadow_sampler);
    if (m_shadow_tex_view.id) sg_destroy_view(m_shadow_tex_view);
    if (m_shadow_color_tex_view.id) sg_destroy_view(m_shadow_color_tex_view);
    if (m_shadow_depth_att_view.id) sg_destroy_view(m_shadow_depth_att_view);
    if (m_shadow_color_att_view.id) sg_destroy_view(m_shadow_color_att_view);
    if (m_shadow_depth.id) sg_destroy_image(m_shadow_depth);
    if (m_shadow_color.id) sg_destroy_image(m_shadow_color);
    if (m_shadow_pipeline.id) sg_destroy_pipeline(m_shadow_pipeline);
    if (m_shadow_shader.id) sg_destroy_shader(m_shadow_shader);
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

void Renderer::SetFieldBounds(bool has_bounds, float half_x, float half_z) {
    m_has_field_bounds = has_bounds;
    if (has_bounds && half_x > 0.0f && half_z > 0.0f) {
        m_field_half_x = half_x;
        m_field_half_z = half_z;
    }
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

void Renderer::BuildLightVPMatrix(float out[16]) const {
    float eye[3] = {0.0f, 10.0f, 0.0f};
    float center[3] = {0.0f, 0.0f, 0.0f};
    float up[3] = {0.0f, 0.0f, -1.0f};
    float view[16], proj[16];
    mat4_look_at(eye, center, up, view);
    // Margin accounts for robots/game pieces extending past the nominal field
    // bounds, and avoids clipping geometry right at the playing surface edge.
    const float margin = 2.0f;
    float half_x = m_field_half_x + margin;
    float half_z = m_field_half_z + margin;
    mat4_ortho(-half_x, half_x, -half_z, half_z, 0.5f, 20.0f, proj);
    mat4_mul(proj, view, out);
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
            if (e->mouse_button == SAPP_MOUSEBUTTON_LEFT) {
                m_mouse_down = true;
                m_mouse_last_x = e->mouse_x;
                m_mouse_last_y = e->mouse_y;
            }
            break;
        case SAPP_EVENTTYPE_MOUSE_UP:
            if (e->mouse_button == SAPP_MOUSEBUTTON_LEFT) m_mouse_down = false;
            break;
        case SAPP_EVENTTYPE_MOUSE_MOVE:
            if (m_mouse_down && !m_cameraLocked) {
                float dx = e->mouse_x - m_mouse_last_x;
                float dy = e->mouse_y - m_mouse_last_y;
                m_mouse_last_x = e->mouse_x;
                m_mouse_last_y = e->mouse_y;
                m_cam.yaw += dx * 0.2f;
                m_cam.pitch -= dy * 0.2f;
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

    // Light VP matrix
    float light_vp[16];
    BuildLightVPMatrix(light_vp);

    // Per-frame FS uniforms
    FsUniforms fs_ub = {};
    fs_ub.light_pos[0] = 0.0f;
    fs_ub.light_pos[1] = 10.0f;
    fs_ub.light_pos[2] = 0.0f;
    fs_ub.light_pos[3] = 1.0f;
    fs_ub.ambient[0] = 0.50f;
    fs_ub.ambient[1] = 0.50f;
    fs_ub.ambient[2] = 0.50f;
    fs_ub.ambient[3] = 1.0f;
    fs_ub.view_pos[0] = m_cam.pos[0];
    fs_ub.view_pos[1] = m_cam.pos[1];
    fs_ub.view_pos[2] = m_cam.pos[2];
    fs_ub.view_pos[3] = 1.0f;
    fs_ub.light_power = 2.5f;
    memcpy(fs_ub.light_vp, light_vp, sizeof(light_vp));

    // ── Shadow pass ───────────────────────────────────────────────────────
    {
        sg_pass shadow_pass = {};
        shadow_pass.action.colors[0].load_action   = SG_LOADACTION_CLEAR;
        shadow_pass.action.colors[0].store_action  = SG_STOREACTION_STORE;
        shadow_pass.action.colors[0].clear_value   = {1.0f, 1.0f, 1.0f, 1.0f};
        shadow_pass.action.depth.load_action       = SG_LOADACTION_CLEAR;
        shadow_pass.action.depth.store_action      = SG_STOREACTION_DONTCARE;
        shadow_pass.action.depth.clear_value       = 1.0f;
        shadow_pass.attachments.colors[0]          = m_shadow_color_att_view;
        shadow_pass.attachments.depth_stencil      = m_shadow_depth_att_view;
        sg_begin_pass(&shadow_pass);

        sg_apply_pipeline(m_shadow_pipeline);

        ShadowVsUniforms svs_ub;
        memcpy(svs_ub.light_vp, light_vp, sizeof(light_vp));

        for (const auto& body : snapshot.bodies) {
            float model[16];
            BuildMatrix(model, body.pos, body.rot);
            memcpy(svs_ub.model, model, sizeof(model));
            sg_apply_uniforms(0, SG_RANGE(svs_ub));

            const CachedMesh* mesh = m_mesh_cache.Get(body.def);
            if (mesh && mesh->valid) {
                sg_bindings bind = {};
                bind.vertex_buffers[0] = mesh->vertex_buf;
                bind.index_buffer = mesh->index_buf;
                sg_apply_bindings(&bind);
                sg_draw(0, mesh->num_indices, 1);
            }
        }

        sg_end_pass();
    }

    // ── Main render pass ──────────────────────────────────────────────────
    {
        sg_pass pass = {};
        pass.action = m_pass_action;
        pass.swapchain = sglue_swapchain();
        sg_begin_pass(&pass);

        sg_apply_pipeline(m_pipeline);

        for (const auto& body : snapshot.bodies) {
            float model[16];
            BuildMatrix(model, body.pos, body.rot);
            VsUniforms vs_ub;
            memcpy(vs_ub.model, model, sizeof(model));
            memcpy(vs_ub.view, view, sizeof(view));
            memcpy(vs_ub.projection, proj, sizeof(proj));
            sg_apply_uniforms(0, SG_RANGE(vs_ub));

            const CachedMesh* mesh = m_mesh_cache.Get(body.def);
            if (mesh && mesh->valid) {
                sg_bindings bind = {};
                bind.vertex_buffers[0] = mesh->vertex_buf;
                bind.index_buffer = mesh->index_buf;
                bind.views[0] = m_shadow_color_tex_view;
                bind.samplers[0] = m_shadow_sampler;

                if (!mesh->ranges.empty()) {
                    for (const auto& range : mesh->ranges) {
                        memcpy(fs_ub.model_color, range.color, sizeof(float[4]));
                        sg_apply_uniforms(1, SG_RANGE(fs_ub));
                        sg_apply_bindings(&bind);
                        sg_draw(range.index_offset, range.index_count, 1);
                    }
                } else {
                    float col[4];
                    BodyColor(body.def, col);
                    if (mesh->has_vertex_colors) {
                        col[0] = 1.0f; col[1] = 1.0f; col[2] = 1.0f; col[3] = 1.0f;
                    }
                    memcpy(fs_ub.model_color, col, sizeof(col));
                    sg_apply_uniforms(1, SG_RANGE(fs_ub));
                    sg_apply_bindings(&bind);
                    sg_draw(0, mesh->num_indices, 1);
                }
            }
        }

        // ── Debug overlay ─────────────────────────────────────────────────
        DrawDebugOverlay(snapshot, nt_connected, sim_hz, target_hz, nt_ping_ms, m_wall_time_offset_ms);

        sg_end_pass();
    }
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