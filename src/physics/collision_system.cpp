#include "lili2d/physics/collision_system.hpp"

#include "lili2d/components/box_collider.hpp"
#include "lili2d/components/dynamic_body.hpp"
#include "lili2d/components/transform.hpp"
#include "lili2d/components/velocity.hpp"
#include "lili2d/ecs/ecs_view.hpp"
#include "lili2d/geometry/vec2.hpp"
#include "lili2d/physics/aabb_collider.hpp"
#include "physics/collision_helper.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace lili {

namespace {

using detail::BlockedSide;
using detail::computeOverlap;
using detail::detectTouchingStaticSides;
using detail::isBlockedInDirection;
using detail::makeWorldAABB;
using detail::sideInDirection;
using detail::sweepAABB;
using detail::SweepHit;

struct Mover
{
    cTransform* transform;
    cVelocity* velocity;
    const cBoxCollider* collider;
    cDynamicBody* body;
    uint8_t blocked_sides = BlockedSide::NONE;
};

[[nodiscard]] float
inverseMass(const cDynamicBody* body) noexcept
{
    return (body && body->mass > 0.0f) ? 1.0f / body->mass : 1.0f;
}

void
resolveAgainstStatic(
    Mover& mover,
    const std::vector<AABB2>& static_boxes,
    float dt
) noexcept
{
    // 1. Push out of any static box we are already overlapping.
    for (const AABB2& wall : static_boxes) {
        Vec2 push_normal;
        float overlap_depth = 0.0f;
        AABB2 mover_box = makeWorldAABB(*mover.transform, *mover.collider);
        if (!computeOverlap(mover_box, wall, push_normal, overlap_depth))
            continue;

        mover.transform->pos += push_normal * overlap_depth;
        mover.blocked_sides |= sideInDirection(-push_normal);

        Vec2 velocity(mover.velocity->x, mover.velocity->y);
        float speed_along_normal = velocity.dot(push_normal);
        if (speed_along_normal < 0.0f) {
            velocity -= push_normal * speed_along_normal;
            mover.velocity->x = velocity.x;
            mover.velocity->y = velocity.y;
        }
        if (push_normal.y < 0.0f && mover.body)
            mover.body->grounded = true;
    }

    // 2. Sweep along this tick's movement (up to 3 passes for corners).
    for (int pass = 0; pass < 3; ++pass) {
        AABB2 mover_box = makeWorldAABB(*mover.transform, *mover.collider);
        Vec2 movement_delta = Vec2(mover.velocity->x, mover.velocity->y) * dt;

        SweepHit earliest_hit;
        for (const AABB2& wall : static_boxes) {
            SweepHit candidate = sweepAABB(mover_box, movement_delta, wall);
            if (candidate.hit &&
                candidate.hit_fraction < earliest_hit.hit_fraction)
                earliest_hit = candidate;
        }
        if (!earliest_hit.hit)
            break;

        Vec2 velocity(mover.velocity->x, mover.velocity->y);
        float speed_along_normal = velocity.dot(earliest_hit.normal);
        if (speed_along_normal >= 0.0f)
            break;

        mover.blocked_sides |= sideInDirection(-earliest_hit.normal);
        if (earliest_hit.normal.y < 0.0f && mover.body)
            mover.body->grounded = true;

        float bounciness = mover.body ? mover.body->bounciness : 0.0f;
        if (bounciness > 0.0f) {
            mover.transform->pos +=
                earliest_hit.normal *
                (speed_along_normal * dt * earliest_hit.hit_fraction);
            velocity -= earliest_hit.normal *
                        ((1.0f + bounciness) * speed_along_normal);
        } else {
            velocity -=
                earliest_hit.normal *
                (speed_along_normal * (1.0f - earliest_hit.hit_fraction));
        }

        if (mover.body) {
            Vec2 tangent(-earliest_hit.normal.y, earliest_hit.normal.x);
            float dot_res = velocity.dot(tangent);

            if (mover.body->damping.x > 0.0f) {
                float damping_factor_x = std::exp(-mover.body->damping.x * dt);
                velocity.x -= tangent.x * (dot_res * (1.0f - damping_factor_x));
            }
            if (mover.body->damping.y > 0.0f) {
                float damping_factor_y = std::exp(-mover.body->damping.y * dt);
                velocity.y -= tangent.y * (dot_res * (1.0f - damping_factor_y));
            }
        }

        mover.velocity->x = velocity.x;
        mover.velocity->y = velocity.y;
    }
}

void
applyPairImpulse(
    Mover& mover_a,
    Mover& mover_b,
    Vec2 push_normal,
    float impulse_scale,
    float share_a,
    float share_b
) noexcept
{
    Vec2 relative_velocity(
        mover_a.velocity->x - mover_b.velocity->x,
        mover_a.velocity->y - mover_b.velocity->y
    );
    float approach_speed = relative_velocity.dot(push_normal);
    if (approach_speed >= 0.0f)
        return;

    float velocity_change = -impulse_scale * approach_speed;
    mover_a.velocity->x += push_normal.x * (velocity_change * share_a);
    mover_a.velocity->y += push_normal.y * (velocity_change * share_a);
    mover_b.velocity->x -= push_normal.x * (velocity_change * share_b);
    mover_b.velocity->y -= push_normal.y * (velocity_change * share_b);
}

void
resolveDynamicPair(Mover& mover_a, Mover& mover_b, float dt) noexcept
{
    AABB2 box_a = makeWorldAABB(*mover_a.transform, *mover_a.collider);
    AABB2 box_b = makeWorldAABB(*mover_b.transform, *mover_b.collider);

    // Compute each body's share of the push along push_normal (push_normal
    // pushes A forward and B backward; a body pinned against a static wall in
    // its push direction gets 0 share, like infinite mass).
    auto compute_shares =
        [&](Vec2 push_normal, float& share_a, float& share_b) -> bool {
        bool a_blocked =
            isBlockedInDirection(mover_a.blocked_sides, push_normal);
        bool b_blocked =
            isBlockedInDirection(mover_b.blocked_sides, -push_normal);

        // Propagate pinning through chains of touching dynamic bodies.
        if (b_blocked)
            mover_a.blocked_sides |= sideInDirection(-push_normal);
        if (a_blocked)
            mover_b.blocked_sides |= sideInDirection(push_normal);

        float inv_mass_a = a_blocked ? 0.0f : inverseMass(mover_a.body);
        float inv_mass_b = b_blocked ? 0.0f : inverseMass(mover_b.body);
        float inv_mass_sum = inv_mass_a + inv_mass_b;
        if (inv_mass_sum <= 0.0f)
            return false;

        share_a = inv_mass_a / inv_mass_sum;
        share_b = inv_mass_b / inv_mass_sum;
        return true;
    };

    // 1. If already overlapping, separate them using effective mass shares.
    Vec2 push_normal;
    float overlap_depth = 0.0f;
    if (computeOverlap(box_a, box_b, push_normal, overlap_depth)) {
        if (push_normal.y < 0.0f && mover_a.body)
            mover_a.body->grounded = true;
        if (push_normal.y > 0.0f && mover_b.body)
            mover_b.body->grounded = true;

        float share_a = 0.0f;
        float share_b = 0.0f;
        if (!compute_shares(push_normal, share_a, share_b))
            return;

        mover_a.transform->pos += push_normal * (overlap_depth * share_a);
        mover_b.transform->pos -= push_normal * (overlap_depth * share_b);

        applyPairImpulse(mover_a, mover_b, push_normal, 1.0f, share_a, share_b);
        return;
    }

    // 2. Otherwise sweep A relative to B to catch impacts before they overlap.
    Vec2 relative_velocity(
        mover_a.velocity->x - mover_b.velocity->x,
        mover_a.velocity->y - mover_b.velocity->y
    );
    SweepHit sweep_hit = sweepAABB(box_a, relative_velocity * dt, box_b);
    if (!sweep_hit.hit || relative_velocity.dot(sweep_hit.normal) >= 0.0f)
        return;

    if (sweep_hit.normal.y < 0.0f && mover_a.body)
        mover_a.body->grounded = true;
    if (sweep_hit.normal.y > 0.0f && mover_b.body)
        mover_b.body->grounded = true;

    float share_a = 0.0f;
    float share_b = 0.0f;
    if (!compute_shares(sweep_hit.normal, share_a, share_b))
        return;

    float bounciness = std::max(
        mover_a.body ? mover_a.body->bounciness : 0.0f,
        mover_b.body ? mover_b.body->bounciness : 0.0f
    );
    if (bounciness > 0.0f) {
        Vec2 velocity_a(mover_a.velocity->x, mover_a.velocity->y);
        Vec2 velocity_b(mover_b.velocity->x, mover_b.velocity->y);
        if (share_a > 0.0f) {
            mover_a.transform->pos +=
                sweep_hit.normal * (velocity_a.dot(sweep_hit.normal) * dt *
                                    sweep_hit.hit_fraction);
        }
        if (share_b > 0.0f) {
            mover_b.transform->pos +=
                sweep_hit.normal * (velocity_b.dot(sweep_hit.normal) * dt *
                                    sweep_hit.hit_fraction);
        }
    }

    float impulse_scale = (bounciness > 0.0f) ? (1.0f + bounciness)
                                              : (1.0f - sweep_hit.hit_fraction);
    applyPairImpulse(
        mover_a, mover_b, sweep_hit.normal, impulse_scale, share_a, share_b
    );

    float damp_x = std::max(
        mover_a.body ? mover_a.body->damping.x : 0.0f,
        mover_b.body ? mover_b.body->damping.x : 0.0f
    );
    float damp_y = std::max(
        mover_a.body ? mover_a.body->damping.y : 0.0f,
        mover_b.body ? mover_b.body->damping.y : 0.0f
    );
    if (damp_x > 0.0f || damp_y > 0.0f) {
        Vec2 tangent(-sweep_hit.normal.y, sweep_hit.normal.x);
        float tangent_speed = relative_velocity.dot(tangent);
        if (damp_x > 0.0f) {
            float tangent_change_x =
                -tangent_speed * (1.0f - std::exp(-damp_x * dt));
            mover_a.velocity->x += tangent.x * (tangent_change_x * share_a);
            mover_b.velocity->x -= tangent.x * (tangent_change_x * share_b);
        }
        if (damp_y > 0.0f) {
            float tangent_change_y =
                -tangent_speed * (1.0f - std::exp(-damp_y * dt));
            mover_a.velocity->y += tangent.y * (tangent_change_y * share_a);
            mover_b.velocity->y -= tangent.y * (tangent_change_y * share_b);
        }
    }
}

} // namespace

void
CollisionSystem::fixedUpdate(ECSRegistry& registry, float dt)
{
    if (dt <= 0.0f)
        return;

    ECSView<cDynamicBody>(registry).each([](Entity, cDynamicBody& body) {
        body.grounded = false;
    });

    std::vector<AABB2> static_boxes;
    ECSView<cTransform, cBoxCollider>(registry).each(
        [&](Entity entity,
            const cTransform& transform,
            const cBoxCollider& collider) {
            if (collider.physical && !registry.hasComponent<cVelocity>(entity))
                static_boxes.push_back(makeWorldAABB(transform, collider));
        }
    );

    std::vector<Mover> movers;
    ECSView<cTransform, cVelocity, cBoxCollider>(registry).each(
        [&](Entity entity,
            cTransform& transform,
            cVelocity& velocity,
            const cBoxCollider& collider) {
            if (!collider.physical)
                return;
            auto* body = registry.hasComponent<cDynamicBody>(entity)
                             ? &registry.getComponent<cDynamicBody>(entity)
                             : nullptr;
            movers.push_back(
                { &transform, &velocity, &collider, body, BlockedSide::NONE }
            );
        }
    );
    if (movers.empty())
        return;

    if (!static_boxes.empty()) {
        for (Mover& mover : movers) {
            resolveAgainstStatic(mover, static_boxes, dt);
            mover.blocked_sides |= detectTouchingStaticSides(
                makeWorldAABB(*mover.transform, *mover.collider), static_boxes
            );
        }
    }

    constexpr int solver_iterations = 4;
    for (int iteration = 0; iteration < solver_iterations; ++iteration) {
        for (size_t i = 0; i < movers.size(); ++i) {
            for (size_t j = i + 1; j < movers.size(); ++j)
                resolveDynamicPair(movers[i], movers[j], dt);
        }

        if (!static_boxes.empty()) {
            for (Mover& mover : movers) {
                resolveAgainstStatic(mover, static_boxes, dt);
                mover.blocked_sides |= detectTouchingStaticSides(
                    makeWorldAABB(*mover.transform, *mover.collider),
                    static_boxes
                );
            }
        }
    }
}

} // namespace lili
