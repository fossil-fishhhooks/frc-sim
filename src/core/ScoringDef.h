#pragma once
#include <string>

struct ScoringZoneDef {
    std::string id;
    int         team;           // 0 or 1
    int         points;
    float       center[3];
    float       half_extents[3];
    float       active_start;   // seconds into match, -1 = always
    float       active_end;     // seconds into match, -1 = always
    bool        pass_through;   // false = piece must come to rest inside

    bool  has_reset_pos = false;
    float reset_pos[3]  = {};
};