#pragma once

#include "lili2d/components/box_collider.hpp"
#include "lili2d/components/transform.hpp"
#include "lili2d/geometry/vec2.hpp"
#include "lili2d/physics/aabb_collider.hpp"

#include <cstdint>
#include <vector>

namespace lili::detail {

enum BlockedSide : uint8_t
{
    NONE = 0,
    TOP = 1 << 0,    // blocked from moving up (-Y)
    BOTTOM = 1 << 1, // blocked from moving down (+Y)
    LEFT = 1 << 2,   // blocked from moving left (-X)
    RIGHT = 1 << 3,  // blocked from moving right (+X)
};

struct SweepHit
{
    bool hit = false;
    float hit_fraction = 1.0f;
    Vec2 normal = {};
};

[[nodiscard]] uint8_t
sideInDirection(Vec2 direction) noexcept;

[[nodiscard]] bool
isBlockedInDirection(uint8_t blocked_sides, Vec2 direction) noexcept;

[[nodiscard]] AABB2
makeWorldAABB(
    const cTransform& transform,
    const cBoxCollider& collider
) noexcept;

[[nodiscard]] uint8_t
detectTouchingStaticSides(
    const AABB2& mover_box,
    const std::vector<AABB2>& static_boxes
) noexcept;

[[nodiscard]] SweepHit
sweepAABB(
    const AABB2& moving_box,
    Vec2 movement_delta,
    const AABB2& target_box
) noexcept;

[[nodiscard]] bool
computeOverlap(
    const AABB2& box_a,
    const AABB2& box_b,
    Vec2& push_normal,
    float& overlap_depth
) noexcept;

} // namespace lili::detail
