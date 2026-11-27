#include "physics/collision_helper.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace lili::detail {

namespace {

[[nodiscard]] bool
sweepAxis(
    float ray_origin,
    float ray_delta,
    float slab_min,
    float slab_max,
    float& entry_time,
    float& exit_time
) noexcept
{
    if (ray_delta == 0.0f)
        return ray_origin > slab_min && ray_origin < slab_max;

    entry_time = (slab_min - ray_origin) / ray_delta;
    exit_time = (slab_max - ray_origin) / ray_delta;
    if (entry_time > exit_time)
        std::swap(entry_time, exit_time);
    return true;
}

} // namespace

uint8_t
sideInDirection(Vec2 direction) noexcept
{
    if (direction.x > 0.0f)
        return BlockedSide::RIGHT;
    if (direction.x < 0.0f)
        return BlockedSide::LEFT;
    if (direction.y > 0.0f)
        return BlockedSide::BOTTOM;
    if (direction.y < 0.0f)
        return BlockedSide::TOP;
    return BlockedSide::NONE;
}

bool
isBlockedInDirection(uint8_t blocked_sides, Vec2 direction) noexcept
{
    return (blocked_sides & sideInDirection(direction)) != 0;
}

AABB2
makeWorldAABB(
    const cTransform& transform,
    const cBoxCollider& collider
) noexcept
{
    auto apply_scale = [](Vec2 value, Vec2 scale) -> Vec2 {
        return { value.x * std::abs(scale.x), value.y * std::abs(scale.y) };
    };
    Vec2 world_size = apply_scale(collider.size, transform.scale);
    Vec2 world_offset = apply_scale(collider.offset, transform.scale);
    Vec2 top_left = (transform.pos + world_offset) - world_size * 0.5f;
    return AABB2(top_left, world_size);
}

uint8_t
detectTouchingStaticSides(
    const AABB2& mover_box,
    const std::vector<AABB2>& static_boxes
) noexcept
{
    constexpr float touch_epsilon = 1e-3f;
    uint8_t blocked = BlockedSide::NONE;

    for (const AABB2& wall : static_boxes) {
        bool overlaps_y = mover_box.min.y < wall.max.y - touch_epsilon &&
                          mover_box.max.y > wall.min.y + touch_epsilon;
        if (overlaps_y) {
            if (std::abs(mover_box.max.x - wall.min.x) <= touch_epsilon)
                blocked |= BlockedSide::RIGHT;
            if (std::abs(mover_box.min.x - wall.max.x) <= touch_epsilon)
                blocked |= BlockedSide::LEFT;
        }

        bool overlaps_x = mover_box.min.x < wall.max.x - touch_epsilon &&
                          mover_box.max.x > wall.min.x + touch_epsilon;
        if (overlaps_x) {
            if (std::abs(mover_box.max.y - wall.min.y) <= touch_epsilon)
                blocked |= BlockedSide::BOTTOM;
            if (std::abs(mover_box.min.y - wall.max.y) <= touch_epsilon)
                blocked |= BlockedSide::TOP;
        }
    }

    return blocked;
}

SweepHit
sweepAABB(
    const AABB2& moving_box,
    Vec2 movement_delta,
    const AABB2& target_box
) noexcept
{
    if (movement_delta.x == 0.0f && movement_delta.y == 0.0f)
        return {};

    Vec2 center = (moving_box.min + moving_box.max) * 0.5f;
    Vec2 half_size = (moving_box.max - moving_box.min) * 0.5f;
    Vec2 expanded_min = target_box.min - half_size;
    Vec2 expanded_max = target_box.max + half_size;

    constexpr float inf = std::numeric_limits<float>::infinity();
    float enter_x = -inf, exit_x = inf;
    float enter_y = -inf, exit_y = inf;

    if (!sweepAxis(
            center.x,
            movement_delta.x,
            expanded_min.x,
            expanded_max.x,
            enter_x,
            exit_x
        ) ||
        !sweepAxis(
            center.y,
            movement_delta.y,
            expanded_min.y,
            expanded_max.y,
            enter_y,
            exit_y
        ))
        return {};

    float entry_time = std::max(enter_x, enter_y);
    float exit_time = std::min(exit_x, exit_y);
    if (entry_time >= exit_time || exit_time <= 0.0f || entry_time < 0.0f ||
        entry_time >= 1.0f)
        return {};

    Vec2 hit_normal =
        (enter_x > enter_y)
            ? Vec2((movement_delta.x > 0.0f) ? -1.0f : 1.0f, 0.0f)
            : Vec2(0.0f, (movement_delta.y > 0.0f) ? -1.0f : 1.0f);
    return { true, entry_time, hit_normal };
}

bool
computeOverlap(
    const AABB2& box_a,
    const AABB2& box_b,
    Vec2& push_normal,
    float& overlap_depth
) noexcept
{
    if (!box_a.overlaps(box_b))
        return false;

    float overlap_x =
        std::min(box_a.max.x, box_b.max.x) - std::max(box_a.min.x, box_b.min.x);
    float overlap_y =
        std::min(box_a.max.y, box_b.max.y) - std::max(box_a.min.y, box_b.min.y);
    Vec2 center_diff = (box_a.min + box_a.max) - (box_b.min + box_b.max);

    if (overlap_x < overlap_y) {
        push_normal = { (center_diff.x < 0.0f) ? -1.0f : 1.0f, 0.0f };
        overlap_depth = overlap_x;
    } else {
        push_normal = { 0.0f, (center_diff.y < 0.0f) ? -1.0f : 1.0f };
        overlap_depth = overlap_y;
    }
    return true;
}

} // namespace lili::detail
