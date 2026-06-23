#include "render/Renderer.h"
#include "render/BodyDraw.h"
#include "render/DebugOverlay.h"
#include "io/EasyLog.h"

#include <sokol_app.h>
#include <sokol_time.h>
#include <sokol_glue.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <thread>


// ── Swapchain format helper ─────────────────────────────────────────────────
// The window's actual swapchain color format is negotiated at startup and
// varies by backend/platform/driver (e.g. RGBA8 vs BGRA8) -- it is NOT
// always SG_PIXELFORMAT_RGBA8. Any pipeline that renders directly into the
// swapchain (as opposed to an offscreen render target the app controls
// itself, like the shadow map) must match whatever sapp_color_format()
// actually reports, or sg_apply_pipeline's validation will reject it.
static sg_pixel_format SwapchainColorFormat() {
    switch (sapp_color_format()) {
    case SAPP_PIXELFORMAT_RGBA8:   return SG_PIXELFORMAT_RGBA8;
    case SAPP_PIXELFORMAT_SRGB8A8: return SG_PIXELFORMAT_SRGB8A8;
    case SAPP_PIXELFORMAT_BGRA8:   return SG_PIXELFORMAT_BGRA8;
    default:
        LOG_ERROR("Renderer: unexpected sapp_color_format(), falling back to RGBA8");
        return SG_PIXELFORMAT_RGBA8;
    }
}

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

// Each backend's shader source lives in its own assets/shader/<backend>/
// subfolder, since GL (loose uniforms), Vulkan (GLSL uniform blocks +
// explicit set/binding), and Metal (MSL) all need genuinely different
// shader text, not just a recompiled version of the same source.
static const char* BackendShaderDir() {
#if defined(SOKOL_VULKAN)
    return "vulkan";
#elif defined(SOKOL_METAL)
    return "metal";
#else
    return "gl";
#endif
}

// ── Inline matrix/vector math (no raylib) ────────────────────────────────────

static void mat4_identity(float m[16]) {
    memset(m, 0, 16 * sizeof(float));
    m[0] = m[5] = m[10] = m[15] = 1.0f;
}

static void mat4_mul(const float a[16], const float b[16], float out[16]) {
    float t[16];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float sum = 0;
            for (int k = 0; k < 4; ++k)
                sum += a[k * 4 + i] * b[j * 4 + k];
            t[j * 4 + i] = sum;
        }
    memcpy(out, t, sizeof(t));
}

static void mat4_perspective(float fov_y, float aspect, float zn, float zf, float out[16]) {
    memset(out, 0, 16 * sizeof(float));
    float f = 1.0f / tanf(fov_y * 0.5f);
    out[0] = f / aspect;
    out[5] = f;
    out[10] = (zf + zn) / (zn - zf);
    out[11] = -1.0f;
    out[14] = (2.0f * zf * zn) / (zn - zf);
}

static void mat4_ortho(float left, float right, float bottom, float top, float zn, float zf, float out[16]) {
    memset(out, 0, 16 * sizeof(float));
    out[0] = 2.0f / (right - left);
    out[5] = 2.0f / (top - bottom);
    out[10] = -2.0f / (zf - zn);
    out[12] = -(right + left) / (right - left);
    out[13] = -(top + bottom) / (top - bottom);
    out[14] = -(zf + zn) / (zf - zn);
    out[15] = 1.0f;
}

static void mat4_look_at(const float eye[3], const float center[3], const float up[3], float out[16]) {
    float f[3], s[3], u[3];
    f[0] = center[0] - eye[0]; f[1] = center[1] - eye[1]; f[2] = center[2] - eye[2];
    float flen = sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    if (flen > 1e-8f) { f[0] /= flen; f[1] /= flen; f[2] /= flen; }

    s[0] = f[1] * up[2] - f[2] * up[1];
    s[1] = f[2] * up[0] - f[0] * up[2];
    s[2] = f[0] * up[1] - f[1] * up[0];
    float slen = sqrtf(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
    if (slen > 1e-8f) { s[0] /= slen; s[1] /= slen; s[2] /= slen; }

    u[0] = s[1] * f[2] - s[2] * f[1];
    u[1] = s[2] * f[0] - s[0] * f[2];
    u[2] = s[0] * f[1] - s[1] * f[0];

    out[0] = s[0]; out[1] = u[0]; out[2] = -f[0]; out[3] = 0;
    out[4] = s[1]; out[5] = u[1]; out[6] = -f[1]; out[7] = 0;
    out[8] = s[2]; out[9] = u[2]; out[10] = -f[2]; out[11] = 0;
    out[12] = -(s[0] * eye[0] + s[1] * eye[1] + s[2] * eye[2]);
    out[13] = -(u[0] * eye[0] + u[1] * eye[1] + u[2] * eye[2]);
    out[14] = f[0] * eye[0] + f[1] * eye[1] + f[2] * eye[2];
    out[15] = 1;
}

static void vec3_sub(const float a[3], const float b[3], float out[3]) {
    out[0] = a[0] - b[0]; out[1] = a[1] - b[1]; out[2] = a[2] - b[2];
}
static void vec3_add(const float a[3], const float b[3], float out[3]) {
    out[0] = a[0] + b[0]; out[1] = a[1] + b[1]; out[2] = a[2] + b[2];
}
static void vec3_scale(const float a[3], float s, float out[3]) {
    out[0] = a[0] * s; out[1] = a[1] * s; out[2] = a[2] * s;
}
static float vec3_dot(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static void vec3_cross(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}
static void vec3_normalize(const float a[3], float out[3]) {
    float len = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (len > 1e-8f) { out[0] = a[0] / len; out[1] = a[1] / len; out[2] = a[2] / len; }
    else memcpy(out, a, 3 * sizeof(float));
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
    float light_vp[16];
    float light_power;
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
"uniform mat4 light_vp;\n"
"uniform float light_power;\n"
"uniform sampler2DShadow shadow_map;\n"
"in vec3 v_normal;\n"
"in vec3 v_pos;\n"
"in vec4 v_color;\n"
"out vec4 frag_color;\n"
"float ShadowPCF(vec3 world_pos) {\n"
"    vec4 lp = light_vp * vec4(world_pos, 1.0);\n"
"    vec3 proj = lp.xyz / lp.w;\n"
"    proj = proj * 0.5 + 0.5;\n"
"    if (proj.z >= 1.0) return 1.0;\n"
"    vec2 texel = 1.0 / vec2(2048.0);\n"
"    float bias = 0.005;\n"
"    float shadow = 0.0;\n"
"    for (int x = -2; x <= 2; ++x)\n"
"        for (int y = -2; y <= 2; ++y)\n"
"            shadow += texture(shadow_map, vec3(proj.xy + vec2(x, y) * texel, proj.z - bias));\n"
"    return shadow / 25.0;\n"
"}\n"
"void main() {\n"
"    vec3 N = normalize(v_normal);\n"
"    vec3 Lv = light_pos.xyz - v_pos;\n"
"    float dist = length(Lv);\n"
"    vec3 L = Lv / dist;\n"
"    float atten = 1.0 / (1.0 + 0.007 * dist * dist);\n"
"    float diff = max(dot(N, L), 0.0);\n"
"    vec3 base = model_color.rgb * v_color.rgb;\n"
"    vec3 amb = ambient.rgb * base;\n"
"    float shadow = ShadowPCF(v_pos);\n"
"    vec3 diffuse = diff * atten * base * light_power * shadow;\n"
"    vec3 V = normalize(view_pos.xyz - v_pos);\n"
"    vec3 H = normalize(L + V);\n"
"    float spec = pow(max(dot(N, H), 0.0), 32.0) * atten * light_power * shadow;\n"
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
"void main() {}\n";

// ── Vulkan SPIR-V compilation (source loaded from assets/shader/vulkan/) ────
#ifdef SOKOL_VULKAN
#include <shaderc/shaderc.h>
#include <vector>

struct CompiledSpv {
    std::vector<uint32_t> bytecode;
};

static CompiledSpv CompileGLSLtoSPV(const char* source, shaderc_shader_kind kind, const char* name) {
    CompiledSpv result;
    LOG_INFO("Renderer: CompileGLSLtoSPV(%s) start", name);
    shaderc_compiler_t compiler = shaderc_compiler_initialize();
    if (!compiler) {
        LOG_ERROR("Renderer: shaderc_compiler_initialize failed");
        return result;
    }
    shaderc_compile_options_t opts = shaderc_compile_options_initialize();
    shaderc_compilation_result_t cr = shaderc_compile_into_spv(compiler, source, strlen(source), kind, name, "main", opts);
   // LOG_INFO("DEBUG: CompileGLSLtoSPV(%s) compile done\n", name);
    if (shaderc_result_get_compilation_status(cr) != shaderc_compilation_status_success) {
        LOG_ERROR("Renderer: shaderc compilation of %s failed: %s", name, shaderc_result_get_error_message(cr));
        shaderc_result_release(cr);
        shaderc_compile_options_release(opts);
        shaderc_compiler_release(compiler);
       // fprintf(stderr, "DEBUG: CompileGLSLtoSPV(%s) COMPILATION FAILED\n", name);
        return result;
    }
    //fprintf(stderr, "DEBUG: CompileGLSLtoSPV(%s) compile SUCCESS\n", name);
    size_t n = shaderc_result_get_length(cr);
    result.bytecode.resize(n / sizeof(uint32_t));
    memcpy(result.bytecode.data(), shaderc_result_get_bytes(cr), n);
    shaderc_result_release(cr);
    shaderc_compile_options_release(opts);
    shaderc_compiler_release(compiler);
    LOG_INFO("Renderer: CompileGLSLtoSPV(%s) done, %zu bytes SPIR-V", name, n);
    return result;
}
#endif

// ── Init ──────────────────────────────────────────────────────────────────────

void Renderer::Init(int width, int height, const char* title, int target_fps) {
    (void)width; (void)height; (void)title;
    m_target_fps = target_fps;

    m_pass_action = {};
    m_pass_action.colors[0].load_action = SG_LOADACTION_CLEAR;
    m_pass_action.colors[0].clear_value = { 0.11f, 0.11f, 0.11f, 1.0f };
    m_pass_action.depth.load_action = SG_LOADACTION_CLEAR;
    m_pass_action.depth.clear_value = 1.0f;

    // ── Load shader sources from files ──────────────────────────────────────
    // Path layout: assets/shader/<backend>/<file>, since GL, Vulkan, and
    // Metal each need genuinely different shader text (see BackendShaderDir).
    const char* dir = BackendShaderDir();
    std::string main_vs_path, main_fs_path, shadow_vs_path, shadow_fs_path;
#if defined(SOKOL_VULKAN)
    main_vs_path = std::string("assets/shader/") + dir + "/main.vert";
    main_fs_path = std::string("assets/shader/") + dir + "/main.frag";
    shadow_vs_path = std::string("assets/shader/") + dir + "/shadow.vert";
    shadow_fs_path = std::string("assets/shader/") + dir + "/shadow.frag";
#elif defined(SOKOL_METAL)
    // Metal combines vertex+fragment per pass into one .metal file; both
    // "paths" below point at the same file, the entry-point names (set in
    // sg_shader_desc.*.entry below) select which function gets used.
    main_vs_path = std::string("assets/shader/") + dir + "/main.metal";
    main_fs_path = main_vs_path;
    shadow_vs_path = std::string("assets/shader/") + dir + "/shadow.metal";
    shadow_fs_path = shadow_vs_path;
#else
    main_vs_path = std::string("assets/shader/") + dir + "/main.vs";
    main_fs_path = std::string("assets/shader/") + dir + "/main.fs";
    shadow_vs_path = std::string("assets/shader/") + dir + "/shadow.vs";
    shadow_fs_path = std::string("assets/shader/") + dir + "/shadow.fs";
#endif

    std::string vs_main_str, fs_main_str, vs_shadow_str, fs_shadow_str;

    bool loaded = ReadFile(main_vs_path.c_str(), vs_main_str);
    const char* vs_main = loaded ? vs_main_str.c_str() : vs_src_main_fallback;
    if (loaded) LOG_INFO("Renderer: loaded %s", main_vs_path.c_str());
    else LOG_ERROR("Renderer: failed to load %s", main_vs_path.c_str());

    loaded = ReadFile(main_fs_path.c_str(), fs_main_str);
    const char* fs_main = loaded ? fs_main_str.c_str() : fs_src_main_fallback;
    if (loaded) LOG_INFO("Renderer: loaded %s", main_fs_path.c_str());
    else LOG_ERROR("Renderer: failed to load %s", main_fs_path.c_str());

    loaded = ReadFile(shadow_vs_path.c_str(), vs_shadow_str);
    const char* vs_shadow = loaded ? vs_shadow_str.c_str() : vs_src_shadow_fallback;
    if (loaded) LOG_INFO("Renderer: loaded %s", shadow_vs_path.c_str());
    else LOG_ERROR("Renderer: failed to load %s", shadow_vs_path.c_str());

    loaded = ReadFile(shadow_fs_path.c_str(), fs_shadow_str);
    const char* fs_shadow = loaded ? fs_shadow_str.c_str() : fs_src_shadow_fallback;
    if (loaded) LOG_INFO("Renderer: loaded %s", shadow_fs_path.c_str());
    else LOG_ERROR("Renderer: failed to load %s", shadow_fs_path.c_str());

#ifdef SOKOL_VULKAN
    // vs_main/fs_main/vs_shadow/fs_shadow above are now the real GLSL text
    // loaded from assets/shader/vulkan/ -- compile it to SPIR-V.
    CompiledSpv vs_main_spv = CompileGLSLtoSPV(vs_main, shaderc_glsl_vertex_shader, "main.vs");
    CompiledSpv fs_main_spv = CompileGLSLtoSPV(fs_main, shaderc_glsl_fragment_shader, "main.fs");
    CompiledSpv vs_shadow_spv = CompileGLSLtoSPV(vs_shadow, shaderc_glsl_vertex_shader, "shadow.vs");
    CompiledSpv fs_shadow_spv = CompileGLSLtoSPV(fs_shadow, shaderc_glsl_fragment_shader, "shadow.fs");
#endif

    // ── Main shader ───────────────────────────────────────────────────────
    sg_shader_desc shd = {};
#ifdef SOKOL_VULKAN
    // NOTE: SG_RANGE(x) expands to {&x, sizeof(x)} -- correct for a plain
    // struct, but vs_main_spv.bytecode is a std::vector<uint32_t>, so that
    // would capture the vector's own control-block address/size instead of
    // its heap-allocated SPIR-V data. Build the range manually from the
    // vector's actual data pointer and byte length.
    shd.vertex_func.bytecode = sg_range{ vs_main_spv.bytecode.data(), vs_main_spv.bytecode.size() * sizeof(uint32_t) };
    shd.vertex_func.entry = "main";
    shd.fragment_func.bytecode = sg_range{ fs_main_spv.bytecode.data(), fs_main_spv.bytecode.size() * sizeof(uint32_t) };
    shd.fragment_func.entry = "main";
#elif defined(SOKOL_METAL)
    shd.vertex_func.source = vs_main;
    shd.vertex_func.entry = "main_vs";
    shd.fragment_func.source = fs_main;
    shd.fragment_func.entry = "main_fs";
#else
    shd.vertex_func.source = vs_main;
    shd.vertex_func.entry = "main";
    shd.fragment_func.source = fs_main;
    shd.fragment_func.entry = "main";
#endif

    shd.uniform_blocks[0].stage = SG_SHADERSTAGE_VERTEX;
    shd.uniform_blocks[0].size = sizeof(VsUniforms);
    shd.uniform_blocks[0].layout = SG_UNIFORMLAYOUT_NATIVE;
    shd.uniform_blocks[0].spirv_set0_binding_n = 0;
    shd.uniform_blocks[0].msl_buffer_n = 0;
    shd.uniform_blocks[0].glsl_uniforms[0] = { SG_UNIFORMTYPE_MAT4, 1, "model" };
    shd.uniform_blocks[0].glsl_uniforms[1] = { SG_UNIFORMTYPE_MAT4, 1, "view" };
    shd.uniform_blocks[0].glsl_uniforms[2] = { SG_UNIFORMTYPE_MAT4, 1, "projection" };

    shd.uniform_blocks[1].stage = SG_SHADERSTAGE_FRAGMENT;
    shd.uniform_blocks[1].size = sizeof(FsUniforms);
    shd.uniform_blocks[1].layout = SG_UNIFORMLAYOUT_NATIVE;
    shd.uniform_blocks[1].spirv_set0_binding_n = 1;
    shd.uniform_blocks[1].msl_buffer_n = 1;
    shd.uniform_blocks[1].glsl_uniforms[0] = { SG_UNIFORMTYPE_FLOAT4, 1, "model_color" };
    shd.uniform_blocks[1].glsl_uniforms[1] = { SG_UNIFORMTYPE_FLOAT4, 1, "ambient" };
    shd.uniform_blocks[1].glsl_uniforms[2] = { SG_UNIFORMTYPE_FLOAT4, 1, "light_pos" };
    shd.uniform_blocks[1].glsl_uniforms[3] = { SG_UNIFORMTYPE_FLOAT4, 1, "view_pos" };
    shd.uniform_blocks[1].glsl_uniforms[4] = { SG_UNIFORMTYPE_MAT4, 1, "light_vp" };
    shd.uniform_blocks[1].glsl_uniforms[5] = { SG_UNIFORMTYPE_FLOAT, 1, "light_power" };

    shd.views[0].texture.stage = SG_SHADERSTAGE_FRAGMENT;
    shd.views[0].texture.image_type = SG_IMAGETYPE_2D;
#if defined(SOKOL_VULKAN)
    shd.views[0].texture.sample_type = SG_IMAGESAMPLETYPE_FLOAT;
#else
    shd.views[0].texture.sample_type = SG_IMAGESAMPLETYPE_DEPTH;
#endif
    shd.views[0].texture.spirv_set1_binding_n = 0;
    shd.views[0].texture.msl_texture_n = 0;

    shd.samplers[0].stage = SG_SHADERSTAGE_FRAGMENT;
#if defined(SOKOL_VULKAN)
    shd.samplers[0].sampler_type = SG_SAMPLERTYPE_FILTERING;
#else
    shd.samplers[0].sampler_type = SG_SAMPLERTYPE_COMPARISON;
#endif
    shd.samplers[0].spirv_set1_binding_n = 1;
    shd.samplers[0].msl_sampler_n = 0;

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
    pip.colors[0].pixel_format = SwapchainColorFormat();
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
    pip.sample_count = sapp_sample_count();
    pip.shader = m_shader;
    pip.index_type = SG_INDEXTYPE_UINT32;
    pip.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
    pip.depth.write_enabled = true;
    pip.cull_mode = SG_CULLMODE_BACK;
    pip.face_winding = SG_FACEWINDING_CCW;
    //fprintf(stderr, "DEBUG: before sg_make_pipeline (main)\n");
    m_pipeline = sg_make_pipeline(&pip);
    //fprintf(stderr, "DEBUG: after sg_make_pipeline (main), id=%u\n", m_pipeline.id);
    if (!m_pipeline.id) {
        LOG_ERROR("Renderer: failed to create main pipeline");
    }

    // ── Shadow shader ─────────────────────────────────────────────────────
    sg_shader_desc shd_shadow = {};
#ifdef SOKOL_VULKAN
    shd_shadow.vertex_func.bytecode = sg_range{ vs_shadow_spv.bytecode.data(), vs_shadow_spv.bytecode.size() * sizeof(uint32_t) };
    shd_shadow.vertex_func.entry = "main";
    shd_shadow.fragment_func.bytecode = sg_range{ fs_shadow_spv.bytecode.data(), fs_shadow_spv.bytecode.size() * sizeof(uint32_t) };
    shd_shadow.fragment_func.entry = "main";
#elif defined(SOKOL_METAL)
    shd_shadow.vertex_func.source = vs_shadow;
    shd_shadow.vertex_func.entry = "shadow_vs";
    shd_shadow.fragment_func.source = fs_shadow;
    shd_shadow.fragment_func.entry = "shadow_fs";
#else
    shd_shadow.vertex_func.source = vs_shadow;
    shd_shadow.vertex_func.entry = "main";
    shd_shadow.fragment_func.source = fs_shadow;
    shd_shadow.fragment_func.entry = "main";
#endif

    shd_shadow.uniform_blocks[0].stage = SG_SHADERSTAGE_VERTEX;
    shd_shadow.uniform_blocks[0].size = sizeof(ShadowVsUniforms);
    shd_shadow.uniform_blocks[0].layout = SG_UNIFORMLAYOUT_NATIVE;
    shd_shadow.uniform_blocks[0].spirv_set0_binding_n = 0;
    shd_shadow.uniform_blocks[0].msl_buffer_n = 0;
    shd_shadow.uniform_blocks[0].glsl_uniforms[0] = { SG_UNIFORMTYPE_MAT4, 1, "light_vp" };
    shd_shadow.uniform_blocks[0].glsl_uniforms[1] = { SG_UNIFORMTYPE_MAT4, 1, "model" };

    m_shadow_shader = sg_make_shader(&shd_shadow);
    if (!m_shadow_shader.id) {
        LOG_ERROR("Renderer: failed to create shadow shader");
    }
   // fprintf(stderr, "DEBUG: shadow shader id=%u\n", m_shadow_shader.id);

    // ── Shadow pipeline ───────────────────────────────────────────────────
    sg_pipeline_desc spip = {};
    spip.color_count = 1;
    spip.colors[0].pixel_format = SG_PIXELFORMAT_RGBA8;
    spip.colors[0].write_mask = SG_COLORMASK_R;
    spip.depth.pixel_format = SG_PIXELFORMAT_DEPTH;
    spip.layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3;
    spip.layout.attrs[0].buffer_index = 0;
    spip.layout.attrs[0].offset = 0;
    spip.layout.buffers[0].stride = sizeof(Vertex);
    spip.shader = m_shadow_shader;
    spip.index_type = SG_INDEXTYPE_UINT32;
    spip.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
    spip.depth.write_enabled = true;
    spip.sample_count = 1;
    spip.cull_mode = SG_CULLMODE_NONE;
    m_shadow_pipeline = sg_make_pipeline(&spip);
  //  fprintf(stderr, "DEBUG: shadow pipeline id=%u\n", m_shadow_pipeline.id);

    // ── Shadow map depth image ─────────────────────────────────────────────
    sg_image_desc depth_desc = {};
    depth_desc.width = SHADOW_MAP_SIZE;
    depth_desc.height = SHADOW_MAP_SIZE;
    depth_desc.pixel_format = SG_PIXELFORMAT_DEPTH;
    depth_desc.sample_count = 1;
    depth_desc.usage.depth_stencil_attachment = true;
    m_shadow_depth = sg_make_image(&depth_desc);
  //  fprintf(stderr, "DEBUG: shadow depth image id=%u\n", m_shadow_depth.id);

    sg_image_desc col_desc = {};
    col_desc.width = SHADOW_MAP_SIZE;
    col_desc.height = SHADOW_MAP_SIZE;
    col_desc.pixel_format = SG_PIXELFORMAT_RGBA8;
    col_desc.sample_count = 1;
    col_desc.usage.color_attachment = true;
    m_shadow_color = sg_make_image(&col_desc);
    fprintf(stderr, "DEBUG: shadow color image id=%u\n", m_shadow_color.id);

    sg_view_desc cav = {};
    cav.color_attachment.image = m_shadow_color;
    m_shadow_color_att_view = sg_make_view(&cav);

    // ── Shadow pass depth attachment view ─────────────────────────────────
    sg_view_desc dav = {};
    dav.depth_stencil_attachment.image = m_shadow_depth;
    m_shadow_depth_att_view = sg_make_view(&dav);

    // ── Shadow map texture view (sampled in main pass) ────────────────────
    sg_view_desc tv = {};
#if defined(SOKOL_VULKAN)
    // Vulkan: sample color attachment (avoids Intel depth→texture GPU hang)
    tv.texture.image = m_shadow_color;
#else
    // GL/Metal: sample depth texture directly via comparison sampler
    tv.texture.image = m_shadow_depth;
#endif
    m_shadow_tex_view = sg_make_view(&tv);

    // ── Shadow sampler ────────────────────────────────────────────────────
    sg_sampler_desc smp_desc = {};
    smp_desc.min_filter = SG_FILTER_LINEAR;
    smp_desc.mag_filter = SG_FILTER_LINEAR;
    smp_desc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
    smp_desc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
#if !defined(SOKOL_VULKAN)
    smp_desc.compare = SG_COMPAREFUNC_LESS_EQUAL;
#endif
    m_shadow_sampler = sg_make_sampler(&smp_desc);

    // ── Camera ───────────────────────────────────────────────────────────
    SetupCamera();
    m_stamp = stm_now();
    m_pace_stamp = stm_now();

    LOG_INFO("Renderer: sokol_gfx initialized");
}

void Renderer::Shutdown() {
    if (m_shadow_sampler.id) sg_destroy_sampler(m_shadow_sampler);
    if (m_shadow_tex_view.id) sg_destroy_view(m_shadow_tex_view);
    if (m_shadow_depth_att_view.id) sg_destroy_view(m_shadow_depth_att_view);
    if (m_shadow_depth.id) sg_destroy_image(m_shadow_depth);
    if (m_shadow_pipeline.id) sg_destroy_pipeline(m_shadow_pipeline);
    if (m_shadow_shader.id) sg_destroy_shader(m_shadow_shader);
    if (m_pipeline.id) sg_destroy_pipeline(m_pipeline);
    if (m_shader.id) sg_destroy_shader(m_shader);
    if (m_shadow_color_att_view.id) sg_destroy_view(m_shadow_color_att_view);
    if (m_shadow_color.id)          sg_destroy_image(m_shadow_color);
    m_mesh_cache.UnloadAll();
    LOG_INFO("Renderer: shutdown");
}

bool Renderer::ShouldClose() const {
    return false;
}


// ── Camera ────────────────────────────────────────────────────────────────────

void Renderer::SetupCamera() {
    float sp = m_cam.dist * sinf(m_cam.pitch * 3.14159265f / 180.0f);
    float c = m_cam.dist * cosf(m_cam.pitch * 3.14159265f / 180.0f);
    m_cam.pos[0] = m_cam.target[0] + c * sinf(m_cam.yaw * 3.14159265f / 180.0f);
    m_cam.pos[1] = m_cam.target[1] - sp;
    m_cam.pos[2] = m_cam.target[2] + c * cosf(m_cam.yaw * 3.14159265f / 180.0f);
}

void Renderer::BuildProjMatrix(float out[16], float fov, float znear, float zfar) const {
    float aspect = (float)sapp_width() / fmaxf(1.0f, (float)sapp_height());
    mat4_perspective(fov * 3.14159265f / 180.0f, aspect, znear, zfar, out);
}

void Renderer::BuildViewMatrix(float out[16]) const {
    mat4_look_at(m_cam.pos, m_cam.target, m_cam.up, out);
}

void Renderer::BuildLightVPMatrix(float out[16]) const {
    float eye[3] = { 0.0f, 10.0f, 0.0f };
    float center[3] = { 0.0f,  0.0f, 0.0f };
    float up[3] = { 0.0f,  0.0f, -1.0f };
    float view[16], proj[16];
    mat4_look_at(eye, center, up, view);
    float hx = m_field_half_extents[0] + 3;
    float hz = m_field_half_extents[1] + 3;
    mat4_ortho(-hx, hx, -hz, hz, 0.5f, 20.0f, proj);

    float vp[16];
    mat4_mul(proj, view, vp);

#ifdef SOKOL_VULKAN
    // Vulkan clip Z is [0,1]; our mat4_ortho produces [-1,1] (GL convention)
    float z_remap[16];
    mat4_identity(z_remap);
    z_remap[10] = 0.5f;
    z_remap[14] = 0.5f;
    mat4_mul(z_remap, vp, out);
#else
    memcpy(out, vp, 64);
#endif
}

void Renderer::UpdateCamera(float dt) {
    float speed = 5.0f * dt;
    float fwd[3], right[3];
    vec3_sub(m_cam.target, m_cam.pos, fwd);
    fwd[1] = 0;
    vec3_normalize(fwd, fwd);
    vec3_cross(fwd, m_cam.up, right);
    vec3_normalize(right, right);

    float move[3] = { 0,0,0 };
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
    // ── Frame pacing (--fps) ────────────────────────────────────────────
    // sokol's frame_cb runs as fast as the swap chain allows; when a target
    // FPS is set we sleep off whatever time is left in the frame budget so
    // we don't render (and re-publish NT/raycast data) faster than asked.
    if (m_target_fps > 0) {
        double frame_budget = 1.0 / (double)m_target_fps;
        double elapsed = stm_sec(stm_now() - m_pace_stamp);
        if (elapsed < frame_budget) {
            double remaining = frame_budget - elapsed;
            std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
        }
    }
    m_pace_stamp = stm_now();

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
    fs_ub.ambient[0] = 0.25f;
    fs_ub.ambient[1] = 0.25f;
    fs_ub.ambient[2] = 0.25f;
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
        shadow_pass.action.depth.load_action = SG_LOADACTION_CLEAR;
        shadow_pass.action.depth.store_action = SG_STOREACTION_STORE;
        shadow_pass.action.depth.clear_value = 1.0f;
        shadow_pass.attachments.colors[0] = m_shadow_color_att_view;
        shadow_pass.attachments.depth_stencil = m_shadow_depth_att_view;
        shadow_pass.action.colors[0].load_action = SG_LOADACTION_DONTCARE;
        shadow_pass.action.colors[0].store_action = SG_STOREACTION_STORE;
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
                bind.views[0] = m_shadow_tex_view;
                bind.samplers[0] = m_shadow_sampler;

                if (!mesh->ranges.empty()) {
                    for (const auto& range : mesh->ranges) {
                        memcpy(fs_ub.model_color, range.color, sizeof(float[4]));
                        sg_apply_uniforms(1, SG_RANGE(fs_ub));
                        sg_apply_bindings(&bind);
                        sg_draw(range.index_offset, range.index_count, 1);
                    }
                }
                else {
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

    
}