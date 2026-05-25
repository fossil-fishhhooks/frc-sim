#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Body/Body.h>

#include <unordered_map>
#include <mutex>
#include <shared_mutex>

// ─────────────────────────────────────────────────────────────────────────────
// ContactListener — tracks normal forces at wheel contact points.
//
// Jolt calls OnContactAdded / OnContactPersisted / OnContactRemoved
// from the physics thread during PhysicsSystem::Update().
//
// ForceApplicator reads GetNormalForce() each tick to clamp tractive force.
// Mutex protects the map since Jolt may call from multiple threads.
// ─────────────────────────────────────────────────────────────────────────────

class ContactListener : public JPH::ContactListener
{
public:
    // Called when two bodies first touch
    void OnContactAdded(const JPH::Body &body1,
                        const JPH::Body &body2,
                        const JPH::ContactManifold &manifold,
                        JPH::ContactSettings &settings) override;

    // Called each step while two bodies remain in contact
    void OnContactPersisted(const JPH::Body &body1,
                            const JPH::Body &body2,
                            const JPH::ContactManifold &manifold,
                            JPH::ContactSettings &settings) override;

    // Called when two bodies separate
    void OnContactRemoved(const JPH::SubShapeIDPair &pair) override;

    // Returns the estimated normal force (N) for a given body.
    // Returns 0.0f if the body has no active contacts.
    // Thread-safe — can be called from ForceApplicator on physics thread.
    float GetNormalForce(JPH::BodyID id) const;

private:
    // Recompute m_body_forces by summing over all live m_contacts entries.
    // Must be called with m_mutex held. Replaces all delta-accumulation logic.
    void RecomputeBodyForce(JPH::BodyID id);

    // contact key: combine two BodyIDs into one 64-bit key
    static uint64_t MakeKey(JPH::BodyID a, JPH::BodyID b)
    {
        uint64_t lo = std::min(a.GetIndexAndSequenceNumber(),
                               b.GetIndexAndSequenceNumber());
        uint64_t hi = std::max(a.GetIndexAndSequenceNumber(),
                               b.GetIndexAndSequenceNumber());
        return (hi << 32) | lo;
    }

    // One entry per body-pair. normal_force is updated each persisted step.
    struct ContactData
    {
        JPH::BodyID id_a;
        JPH::BodyID id_b;
        float normal_force = 0.0f;
        int sub_shape_count = 0; // # of active sub-shape pairs for this body pair
    };

    mutable std::shared_mutex m_mutex;
    std::unordered_map<uint64_t, ContactData> m_contacts;  // body-pair key → data
    std::unordered_map<uint64_t, uint64_t> m_subshape_map; // sub-shape key → body-pair key

    // Per-body accumulated normal force (sum across all contact pairs)
    std::unordered_map<uint32_t, float> m_body_forces;

    // Unique key for a sub-shape pair (differentiates multiple contacts on same body pair)
    // Implemented as a file-static in ContactListener.cpp
};