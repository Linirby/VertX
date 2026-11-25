#include "lili2d/physics/collision_system.hpp"

#include "lili2d/components/box_collider.hpp"
#include "lili2d/components/dynamic_body.hpp"
#include "lili2d/components/transform.hpp"
#include "lili2d/components/velocity.hpp"
#include "lili2d/ecs/ecs_view.hpp"
#include "lili2d/geometry/vec2.hpp"
#include "lili2d/physics/aabb_collider.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

namespace lili {

namespace {

struct SweepHit
{
    bool hit = false;
    float t = 1.0f;
    Vec2 normal = {};
};

[[nodiscard]] AABB2
makeWorldAABB(const cTransform& trans, const cBoxCollider& box) noexcept
{
    auto apply_scale = [](Vec2 target, Vec2 scale) -> Vec2 {
        return { target.x * std::abs(scale.x), target.y * std::abs(scale.y) };
    };
    auto target_size = apply_scale(box.size, trans.scale);
    Vec2 target_offset = apply_scale(box.offset, trans.scale);
    return AABB2((trans.pos + target_offset) - target_size * 0.5f, target_size);
}

[[nodiscard]] bool
sweepAxis(
    float origin,
    float delta,
    float min,
    float max,
    float& t_near,
    float& t_far
) noexcept
{
    if (delta == 0.0f)
        return origin > min && origin < max;

    t_near = (min - origin) / delta;
    t_far = (max - origin) / delta;
    if (t_near > t_far)
        std::swap(t_near, t_far);
    return true;
}

[[nodiscard]] SweepHit
sweepAABB(const AABB2& moving, Vec2 delta, const AABB2& target) noexcept
{
    if (delta.x == 0.0f && delta.y == 0.0f)
        return {};

    Vec2 origin = (moving.min + moving.max) * 0.5f;
    Vec2 half = (moving.max - moving.min) * 0.5f;
    Vec2 exp_min = target.min - half;
    Vec2 exp_max = target.max + half;

    constexpr float inf = std::numeric_limits<float>::infinity();
    float nx = -inf, fx = inf;
    float ny = -inf, fy = inf;

    if (!sweepAxis(origin.x, delta.x, exp_min.x, exp_max.x, nx, fx) ||
        !sweepAxis(origin.y, delta.y, exp_min.y, exp_max.y, ny, fy))
        return {};

    float t_enter = std::max(nx, ny);
    float t_exit = std::min(fx, fy);
    if (t_enter >= t_exit || t_exit <= 0.0f || t_enter < 0.0f ||
        t_enter >= 1.0f)
        return {};

    Vec2 normal = (nx > ny) ? Vec2((delta.x > 0.0f) ? -1.0f : 1.0f, 0.0f)
                            : Vec2(0.0f, (delta.y > 0.0f) ? -1.0f : 1.0f);
    return { true, t_enter, normal };
}

void
resolveOverlap(
    cTransform& trans,
    cVelocity& vel,
    const cBoxCollider& box,
    cDynamicBody* body,
    const AABB2& target
) noexcept
{
    AABB2 a = makeWorldAABB(trans, box);
    if (!a.overlaps(target))
        return;

    float ox =
        std::min(a.max.x, target.max.x) - std::max(a.min.x, target.min.x);
    float oy =
        std::min(a.max.y, target.max.y) - std::max(a.min.y, target.min.y);
    Vec2 diff = (a.min + a.max) - (target.min + target.max);

    bool push_x = ox < oy;
    float& pos_axis = push_x ? trans.pos.x : trans.pos.y;
    float& vel_axis = push_x ? vel.x : vel.y;
    float dir = ((push_x ? diff.x : diff.y) < 0.0f) ? -1.0f : 1.0f;

    pos_axis += dir * (push_x ? ox : oy);
    if (vel_axis * dir < 0.0f)
        vel_axis = 0.0f;
    if (!push_x && dir < 0.0f && body)
        body->grounded = true;
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
        [&](Entity e, const cTransform& trans, const cBoxCollider& box) {
            if (box.physical && !registry.hasComponent<cVelocity>(e))
                static_boxes.push_back(makeWorldAABB(trans, box));
        }
    );
    if (static_boxes.empty())
        return;

    ECSView<cTransform, cVelocity, cBoxCollider>(registry).each(
        [&](
            Entity e, cTransform& trans, cVelocity& vel, const cBoxCollider& box
        ) {
            if (!box.physical)
                return;

            auto* body = registry.hasComponent<cDynamicBody>(e)
                             ? &registry.getComponent<cDynamicBody>(e)
                             : nullptr;

            for (const AABB2& target : static_boxes)
                resolveOverlap(trans, vel, box, body, target);

            for (int i = 0; i < 3; ++i) {
                AABB2 moving = makeWorldAABB(trans, box);
                Vec2 delta = Vec2(vel.x, vel.y) * dt;

                SweepHit earliest;
                for (const AABB2& target : static_boxes) {
                    SweepHit hit = sweepAABB(moving, delta, target);
                    if (hit.hit && hit.t < earliest.t)
                        earliest = hit;
                }
                if (!earliest.hit)
                    break;

                Vec2 target_vel(vel.x, vel.y);
                float dot_prod = target_vel.dot(earliest.normal);
                if (dot_prod >= 0.0f)
                    break;

                if (earliest.normal.y < 0.0f && body)
                    body->grounded = true;

                float bounce = body ? body->bounciness : 0.0f;
                if (bounce > 0.0f) {
                    trans.pos += earliest.normal * (dot_prod * dt * earliest.t);
                    target_vel -=
                        earliest.normal * ((1.0f + bounce) * dot_prod);
                } else
                    target_vel -=
                        earliest.normal * (dot_prod * (1.0f - earliest.t));

                if (body && body->contact_friction > 0.0f) {
                    Vec2 tangent(-earliest.normal.y, earliest.normal.x);
                    float damp = std::exp(-body->contact_friction * dt);
                    target_vel -=
                        tangent * (target_vel.dot(tangent) * (1.0f - damp));
                }

                vel.x = target_vel.x;
                vel.y = target_vel.y;
            }
        }
    );
}

} // namespace lili
