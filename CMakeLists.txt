cmake_minimum_required(VERSION 3.20)
project(frc_sim LANGUAGES CXX)

cmake_policy(SET CMP0091 NEW)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded" CACHE STRING "" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Backend selection ───────────────────────────────────────────────────────────
# Graphics backend, one of: gl, vulkan, d3d11 (Windows), metal (macOS).
# This is a compile-time choice; sokol-gfx does not support runtime switching.
# Use --backend at runtime to match, or the binary will abort.
set(FRC_SIM_BACKEND "gl" CACHE STRING
    "Graphics backend (gl|vulkan|d3d11|metal), platform-dependent")

# ── vcpkg deps ─────────────────────────────────────────────────────────────────
find_package(nlohmann_json CONFIG REQUIRED)

# ── FetchContent ───────────────────────────────────────────────────────────────
include(FetchContent)

set(TARGET_UNIT_TESTS  OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES     OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER      OFF CACHE BOOL "" FORCE)
set(JPH_DEBUG_RENDERER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "" FORCE)
set(JPH_ENABLE_ASSERTS OFF CACHE BOOL "" FORCE)
set(JPH_ENABLE_EXTERNAL_PROFILE OFF CACHE BOOL "" FORCE)
set(JPH_DEBUG_RENDERER_DISABLED ON CACHE BOOL "" FORCE)

FetchContent_Declare(JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG        v5.2.0
    SOURCE_SUBDIR  Build
)
FetchContent_MakeAvailable(JoltPhysics)

FetchContent_Declare(allwpilib
    GIT_REPOSITORY https://github.com/wpilibsuite/allwpilib.git
    GIT_TAG        v2025.3.2
)
set(WITH_JAVA          OFF CACHE BOOL "" FORCE)
set(WITH_CSCORE        OFF CACHE BOOL "" FORCE)
set(WITH_WPIMATH       OFF CACHE BOOL "" FORCE)
set(WITH_WPIUNITS      OFF CACHE BOOL "" FORCE)
set(WITH_WPILIBJ       OFF CACHE BOOL "" FORCE)
set(WITH_WPILIB        OFF CACHE BOOL "" FORCE)
set(WITH_SIMULATION_MODULES OFF CACHE BOOL "" FORCE)
set(WITH_TESTS         OFF CACHE BOOL "" FORCE)
set(WITH_GUI           OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(allwpilib)

# ── Sources ────────────────────────────────────────────────────────────────────
add_executable(frc_sim
    src/main.cpp

    src/core/BodyLoader.cpp
    src/core/MotorRegistry.cpp
    src/core/SceneLoader.cpp
    src/core/SimWorld.cpp
    src/core/SimLoop.cpp
    src/core/ScoreTracker.cpp

    src/physics/ShapeLoader.cpp
    src/physics/ForceApplicator.cpp
    src/physics/ContactListener.cpp
    src/physics/MechanismSystem.cpp

    src/render/Renderer.cpp
    src/render/BodyDraw.cpp
    src/render/DebugOverlay.cpp
    src/render/StreamEncoder.cpp
    src/render/MeshCache.cpp

    src/external/sokol/sokol_impl.cpp
    src/external/cgltf_impl.cpp

    src/io/NTClient.cpp
    src/io/Raycaster.cpp
)

target_include_directories(frc_sim PRIVATE src)
target_include_directories(frc_sim PRIVATE src/external/sokol)

# ── Backend selection ───────────────────────────────────────────────────────────
# Map FRC_SIM_BACKEND to the matching SOKOL_* define and validate platform.
#
# Backend   Platform     Sokol define     Required link libraries
# -------   --------     --------------   ------------------------
# gl        all          SOKOL_GLCORE     GL/OpenGL
# vulkan    all          SOKOL_VULKAN     Vulkan
# d3d11     Win32 only   SOKOL_D3D11      d3d11, dxgi
# metal     Apple only   SOKOL_METAL      Metal.framework
set(_BACKEND_IS_VALID TRUE)

if(FRC_SIM_BACKEND STREQUAL "gl")
    target_compile_definitions(frc_sim PRIVATE SOKOL_GLCORE)
elseif(FRC_SIM_BACKEND STREQUAL "vulkan")
    target_compile_definitions(frc_sim PRIVATE SOKOL_VULKAN)
elseif(FRC_SIM_BACKEND STREQUAL "d3d11")
    if(NOT WIN32)
        set(_BACKEND_IS_VALID FALSE)
        message(FATAL_ERROR "FRC_SIM_BACKEND=d3d11 is only supported on Windows")
    endif()
    target_compile_definitions(frc_sim PRIVATE SOKOL_D3D11)
elseif(FRC_SIM_BACKEND STREQUAL "metal")
    if(NOT APPLE)
        set(_BACKEND_IS_VALID FALSE)
        message(FATAL_ERROR "FRC_SIM_BACKEND=metal is only supported on macOS")
    endif()
    target_compile_definitions(frc_sim PRIVATE SOKOL_METAL)
else()
    message(FATAL_ERROR
        "FRC_SIM_BACKEND must be one of: gl, vulkan, d3d11 (Windows), metal (macOS)")
endif()

# vcpkg installs into a per-platform/arch "triplet" directory; this must
# match whatever triplet vcpkg actually used for this build (visible in
# vcpkg_installed/<triplet>/...). Override with -DFRC_SIM_VCPKG_TRIPLET=...
# if your setup uses something nonstandard (e.g. a static or cross triplet).
if(NOT FRC_SIM_VCPKG_TRIPLET)
    if(WIN32)
        set(FRC_SIM_VCPKG_TRIPLET "x64-windows")
    elseif(APPLE)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
            set(FRC_SIM_VCPKG_TRIPLET "arm64-osx")
        else()
            set(FRC_SIM_VCPKG_TRIPLET "x64-osx")
        endif()
    else()
        set(FRC_SIM_VCPKG_TRIPLET "x64-linux")
    endif()
endif()
target_include_directories(frc_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/build/vcpkg_installed/${FRC_SIM_VCPKG_TRIPLET}/include)

target_link_libraries(frc_sim PRIVATE
    Jolt
    ntcore
    wpiutil
    nlohmann_json::nlohmann_json
)

if(UNIX AND NOT APPLE)
    # ── Linux: X11 is always needed; platform libs ─────────────────────────
    target_link_libraries(frc_sim PRIVATE dl pthread m X11 Xi Xcursor)

    # ── Linux: backend-specific libraries ──────────────────────────────────
    if(FRC_SIM_BACKEND STREQUAL "vulkan")
        target_link_libraries(frc_sim PRIVATE vulkan Xrandr)
    else()
        target_link_libraries(frc_sim PRIVATE GL)
    endif()
endif()

# ── macOS: backend-specific frameworks ────────────────────────────────────────
if(APPLE)
    # Common to all macOS builds:
    target_link_libraries(frc_sim PRIVATE
        "-framework Cocoa"
        "-framework IOKit"
        "-framework QuartzCore"
    )
    if(FRC_SIM_BACKEND STREQUAL "metal")
        target_link_libraries(frc_sim PRIVATE
            "-framework Metal"
            "-framework MetalKit"
        )
    else()
        target_link_libraries(frc_sim PRIVATE "-framework OpenGL")
    endif()
endif()

# ── Windows: backend-specific libraries ──────────────────────────────────────
if(WIN32)
    if(FRC_SIM_BACKEND STREQUAL "d3d11")
        target_link_libraries(frc_sim PRIVATE d3d11 dxgi)
    elseif(FRC_SIM_BACKEND STREQUAL "vulkan")
        target_link_libraries(frc_sim PRIVATE vulkan-1)
    else()
        target_link_libraries(frc_sim PRIVATE opengl32)
    endif()
endif()

target_compile_definitions(frc_sim PRIVATE
    JPH_DEBUG_RENDERER_DISABLED
)

# Copy assets next to the binary on every build
add_custom_command(TARGET frc_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/assets"
        "$<TARGET_FILE_DIR:frc_sim>/assets"
)
