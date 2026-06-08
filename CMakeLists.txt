cmake_minimum_required(VERSION 3.20)
project(frc_sim LANGUAGES CXX)

cmake_policy(SET CMP0091 NEW)

# Default to Release if no build type specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Force release CRT (/MD) globally — including all FetchContent subprojects.
# CMAKE_MSVC_RUNTIME_LIBRARY alone is insufficient because subprojects (Jolt)
# call cmake_policy(CMP0091 NEW) themselves and set their own value.
# CMAKE_USER_MAKE_RULES_OVERRIDE runs last and rewrites the flags directly.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded" CACHE STRING "" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── vcpkg deps ─────────────────────────────────────────────────────────────────
find_package(raylib        CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

# ── FetchContent ───────────────────────────────────────────────────────────────
include(FetchContent)

# Jolt Physics
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




# ntcore (cherry-picked from allwpilib — just ntcore + wpiutil)
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

    src/physics/ShapeLoader.cpp
    src/physics/ForceApplicator.cpp
    src/physics/ContactListener.cpp
    src/physics/MechanismSystem.cpp

    src/render/Renderer.cpp
    src/render/BodyDraw.cpp
    src/render/DebugOverlay.cpp
    src/render/StreamEncoder.cpp

    src/io/NTClient.cpp
)

target_include_directories(frc_sim PRIVATE src)

target_link_libraries(frc_sim PRIVATE
    Jolt
    ntcore
    wpiutil
    raylib
    nlohmann_json::nlohmann_json
)



# Jolt must be compiled without its debug renderer to match our config
target_compile_definitions(frc_sim PRIVATE
    JPH_DEBUG_RENDERER_DISABLED
)

# Force release CRT at the linker level regardless of build type.
# Explicitly ignore all debug CRT libs and replace with release equivalents.
# This is the only reliable fix when FetchContent deps ignore CMAKE_MSVC_RUNTIME_LIBRARY.
if(MSVC)
    target_compile_options(frc_sim PRIVATE /MT)
    target_link_options(frc_sim PRIVATE
        /NODEFAULTLIB:libcmtd.lib
        /NODEFAULTLIB:libcpmtd.lib
        /NODEFAULTLIB:msvcrtd.lib
        /NODEFAULTLIB:msvcprtd.lib
        /NODEFAULTLIB:vcruntimed.lib
        /NODEFAULTLIB:ucrtd.lib
        /NODEFAULTLIB:msvcrt.lib
        /NODEFAULTLIB:msvcprt.lib
        /NODEFAULTLIB:vcruntime.lib
        /NODEFAULTLIB:ucrt.lib
        /DEFAULTLIB:libcmt.lib
        /DEFAULTLIB:libcpmt.lib
        /DEFAULTLIB:libucrt.lib
    )

    # ── raylib / Windows SDK name collision fix ───────────────────────────────
    # wingdi.h declares Rectangle() as a C function; winuser.h declares
    # CloseWindow(), ShowCursor(), DrawText(), LoadImage(), PlaySound().
    # raylib uses the same names for its own API.  These are real function
    # declarations, not macros, so #undef cannot remove them.
    #
    # The only reliable fix: make raylib.h parse BEFORE windows.h in every
    # translation unit.  MSVC's /FI (force-include) injects a header at the
    # very start of every .cpp before any #include the file itself contains,
    # including headers pulled in transitively through wpilib/ntcore.
    #
    # win_raylib_compat.h:
    #   1. #includes raylib.h   ← compiler sees raylib's Rectangle struct first
    #   2. #includes winsock2.h / windows.h
    #   3. #undefs the handful of names that Windows also defines as macros
    #      (DrawText→DrawTextA, LoadImage→LoadImageA, etc.)
    target_compile_options(frc_sim PRIVATE
        /FI "${CMAKE_SOURCE_DIR}/src/render/win_raylib_compat.h"
    )
endif()

# Platform linker extras
if(APPLE)
    target_link_libraries(frc_sim PRIVATE
        "-framework Cocoa"
        "-framework IOKit"
        "-framework OpenGL"
    )
elseif(UNIX)
    target_link_libraries(frc_sim PRIVATE GL dl pthread m ${CMAKE_CURRENT_SOURCE_DIR}/build/vcpkg_installed/x64-linux/lib/libglfw3.a)
endif()

# Copy assets next to the binary on every build
add_custom_command(TARGET frc_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/assets"
        "$<TARGET_FILE_DIR:frc_sim>/assets"
)
