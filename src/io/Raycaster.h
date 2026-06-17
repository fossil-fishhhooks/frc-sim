#pragma once
#include "io/RaycastDef.h"
#include "core/Snapshot.h"

#include <networktables/NetworkTableInstance.h>
#include <networktables/FloatArrayTopic.h>

#include <cmath>
#include <span>
#include <vector>

// Forward declare — no Jolt headers needed in this file
class SimWorld;

struct RayRenderData
{
    float   origin[3];
    float   hit[3];
    bool    did_hit;
    uint8_t color[4];
};

class Raycaster
{
public:
    void Init(const RaycastConfig &cfg,
              nt::NetworkTableInstance &inst,
              int robot_slot);

    void CastAndPublish(const WorldSnapshot &snapshot, SimWorld &world);

    const std::vector<RayRenderData> &RenderData() const { return m_render; }

private:
    const RaycastConfig        *m_cfg        = nullptr;
    int                         m_robot_slot = 0;
    nt::FloatArrayPublisher     m_pub;
    std::vector<float>          m_hits;
    std::vector<RayRenderData>  m_render;
};