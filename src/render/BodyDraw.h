#pragma once
#include "core/Snapshot.h"
#include "core/BodyDef.h"
#include <raylib.h>

// Preload a model into the cache. Safe to call multiple times for same def.
// Must be called after InitWindow(). No-op if mesh_path is empty.
void PreloadMesh(const BodyDef* def);

// Release all cached models. Call before CloseWindow().
void UnloadAllMeshes();

// Draw a single body from snapshot data.
void DrawBodySnapshot(const BodySnapshot& body, Shader* shader, bool wireframe); // OR nullptr = no hsader
