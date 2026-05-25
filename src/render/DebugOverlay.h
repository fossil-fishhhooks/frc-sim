#pragma once
#include "core/Snapshot.h"

void DrawDebugOverlay(const WorldSnapshot &snapshot, bool nt_connected, float sim_hz, float target_hz, float nt_staleness_ms, float wall_time_offset_ms = 0.0f);
