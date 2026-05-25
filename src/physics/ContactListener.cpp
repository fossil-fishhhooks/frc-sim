#include "physics/ContactListener.h"
#include "io/EasyLog.h"

#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/MotionProperties.h>
#include <algorithm>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Normal force model
// ─────────────────────────────────────────────────────────────────────────────
//
// GetNormalForce(id) returns the BODY-LEVEL normal force: mass * g * cos(θ).
// This is the total weight supported by the contact surface — the same value
// regardless of how many sub-shape triangle pairs are touching.
//
// CRITICAL: we use MAX across contact pairs, NOT SUM.
//
// A convex hull robot chassis on a flat floor creates many sub-shape contacts
// (one per colliding triangle edge/vertex pair). Each pair reports the same
// body mass and roughly the same normal. Summing them multiplies m*g by the
// number of contact sub-shapes — easily 10–20×, inflating friction by the
// same factor and giving unrealistically high traction.
//
// Taking the max means: however many contact pairs are active, the body-level
// normal force is m*g*cos(θ) — correct. Multiple contact pairs on different
// surfaces (e.g. floor + wall) take the max of those normals, which still
// gives a sensible bound.
//
// ForceApplicator divides this by grounded_wheel_count to get per-wheel N.
// ─────────────────────────────────────────────────────────────────────────────

static constexpr float GRAVITY = 9.81f;

static float ComputeNormalForce(const JPH::Body &body1,
                                const JPH::Body &body2,
                                const JPH::ContactManifold &manifold)
{
    JPH::Vec3 up(0.0f, 1.0f, 0.0f);
    float cos_theta = std::abs(manifold.mWorldSpaceNormal.Dot(up));

    bool b1_static = body1.IsStatic();
    bool b2_static = body2.IsStatic();

    if (b1_static && !b2_static)
    {
        float mass = 1.0f / body2.GetMotionProperties()->GetInverseMass();
        return mass * GRAVITY * cos_theta;
    }
    if (b2_static && !b1_static)
    {
        float mass = 1.0f / body1.GetMotionProperties()->GetInverseMass();
        return mass * GRAVITY * cos_theta;
    }
    if (!b1_static && !b2_static)
    {
        float m1 = 1.0f / body1.GetMotionProperties()->GetInverseMass();
        float m2 = 1.0f / body2.GetMotionProperties()->GetInverseMass();
        return ((m1 + m2) * 0.5f) * GRAVITY * cos_theta;
    }
    return 0.0f;
}

// ── Sub-shape key ─────────────────────────────────────────────────────────────

static uint64_t MakeSubShapeKey(JPH::SubShapeID s1, JPH::SubShapeID s2,
                                JPH::BodyID b1, JPH::BodyID b2)
{
    uint64_t bk1 = b1.GetIndexAndSequenceNumber();
    uint64_t bk2 = b2.GetIndexAndSequenceNumber();
    uint64_t sk1 = s1.GetValue();
    uint64_t sk2 = s2.GetValue();
    if (bk1 > bk2 || (bk1 == bk2 && sk1 > sk2))
    {
        std::swap(bk1, bk2);
        std::swap(sk1, sk2);
    }
    return (bk1 * 2654435761ULL) ^ (bk2 * 2246822519ULL) ^
           (sk1 * 3266489917ULL) ^ (sk2 * 668265263ULL);
}


// dont full rebuild
void ContactListener::RecomputeBodyForce(JPH::BodyID id)
{
    // Scan only contacts involving this body — not all contacts in the scene
    float best = 0.0f;
    for (const auto &[_, cd] : m_contacts)
    {
        if (cd.id_a == id || cd.id_b == id)
            best = std::max(best, cd.normal_force);
    }

    uint32_t key = id.GetIndexAndSequenceNumber();
    if (best > 0.0f)
        m_body_forces[key] = best;
    else
        m_body_forces.erase(key); // body lifted off, don't leave stale entry
}

// ── Callbacks ─────────────────────────────────────────────────────────────────

void ContactListener::OnContactAdded(const JPH::Body &body1,
                                     const JPH::Body &body2,
                                     const JPH::ContactManifold &manifold,
                                     JPH::ContactSettings & /*settings*/)
{
    float force = ComputeNormalForce(body1, body2, manifold);
    uint64_t body_key = MakeKey(body1.GetID(), body2.GetID());
    uint64_t sub_key = MakeSubShapeKey(manifold.mSubShapeID1, manifold.mSubShapeID2,
                                       body1.GetID(), body2.GetID());

    std::unique_lock<std::shared_mutex> lock(m_mutex);

    m_subshape_map[sub_key] = body_key;

    auto &cd = m_contacts[body_key];
    cd.id_a = body1.GetID();
    cd.id_b = body2.GetID();
    cd.sub_shape_count++;
    cd.normal_force = force; // overwrite, not accumulate

    RecomputeBodyForce(body1.GetID());
    RecomputeBodyForce(body2.GetID());

    LOG_DEBUG("ContactListener: Added body_key=%llu force=%.1fN sub_count=%d",
              body_key, force, cd.sub_shape_count);
}

void ContactListener::OnContactPersisted(const JPH::Body &body1,
                                         const JPH::Body &body2,
                                         const JPH::ContactManifold &manifold,
                                         JPH::ContactSettings & /*settings*/)
{
    float force = ComputeNormalForce(body1, body2, manifold);
    uint64_t body_key = MakeKey(body1.GetID(), body2.GetID());

    std::unique_lock<std::shared_mutex> lock(m_mutex);

    auto it = m_contacts.find(body_key);
    if (it == m_contacts.end())
        return;

    it->second.normal_force = force; // overwrite, not accumulate
    RecomputeBodyForce(body1.GetID());
    RecomputeBodyForce(body2.GetID());

    LOG_DEBUG("ContactListener: Persisted body_key=%llu force=%.1fN", body_key, force);
}

void ContactListener::OnContactRemoved(const JPH::SubShapeIDPair &pair)
{
    uint64_t body_key = MakeKey(pair.GetBody1ID(), pair.GetBody2ID());
    uint64_t sub_key = MakeSubShapeKey(pair.GetSubShapeID1(), pair.GetSubShapeID2(),
                                       pair.GetBody1ID(), pair.GetBody2ID());

    std::unique_lock<std::shared_mutex> lock(m_mutex);

    m_subshape_map.erase(sub_key);

    auto it = m_contacts.find(body_key);
    if (it == m_contacts.end())
        return;

    it->second.sub_shape_count--;
    if (it->second.sub_shape_count <= 0)
    {
        m_contacts.erase(it);
        LOG_DEBUG("ContactListener: Removed body_key=%llu", body_key);
    }

    RecomputeBodyForce(pair.GetBody1ID());
    RecomputeBodyForce(pair.GetBody2ID());
}

float ContactListener::GetNormalForce(JPH::BodyID id) const
{
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_body_forces.find(id.GetIndexAndSequenceNumber());
    return (it != m_body_forces.end()) ? it->second : 0.0f;
}