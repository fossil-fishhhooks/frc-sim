cmake_minimum_required(VERSION 3.20)
project(frc_sim LANGUAGES CXX)

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
set(DEBUG_RENDERER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "" FORCE)

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
