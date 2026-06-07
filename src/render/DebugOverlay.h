#pragma once
#include "core/Snapshot.h"

// nt_connected / nt_ping_ms are aggregated across all robots by caller
void DrawDebugOverlay(const WorldSnapshot &snapshot,
                      bool  nt_connected,
                      float sim_hz,
                      float target_hz,
                      float nt_ping_ms,
                      float wall_time_offset_ms = 0.0f);