#define SOKOL_IMPLEMENTATION
#define SOKOL_GFX_IMPL
#define SOKOL_APP_IMPL
#define SOKOL_GLUE_IMPL
#define SOKOL_TIME_IMPL
#define SOKOL_LOG_IMPL
#define SOKOL_DEBUGTEXT_IMPL

// On Windows, sokol_app.h defaults to a WinMain() entry point (a GUI-
// subsystem convention), which doesn't reliably attach a console to
// stdout/printf even when the linker is told /subsystem:console -- the
// CRT's console wiring depends on which entry symbol is actually present,
// not just the subsystem flag. This app is console-first (LOG_INFO/
// PrintUsage go to stdout, --help is meant to be read in a terminal), so
// force a real main() to match. No effect on non-Windows platforms.
#define SOKOL_WIN32_FORCE_MAIN

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"
#include "sokol_time.h"
#include "sokol_log.h"
#include "sokol_debugtext.h"

// Expose the compiled backend name so the runtime can validate --backend.
extern const char* frc_sim_compiled_backend;
#if defined(SOKOL_METAL)
    const char* frc_sim_compiled_backend = "metal";
#elif defined(SOKOL_VULKAN)
    const char* frc_sim_compiled_backend = "vulkan";
#else
    const char* frc_sim_compiled_backend = "gl";
#endif
