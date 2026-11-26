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

struct Mover
{
    cTransform* trans;
    cVelocity* vel;
    const cBoxCollider* box;
    cDynamicBody* body;
};

[[nodiscard]] float
invMass(const cDynamicBody* body) noexcept
{
    return (body && body->mass > 0.0f) ? 1.0f / body->mass : 1.0f;
}

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

[[nodiscard]] bool
computeOverlap(
    const AABB2& a,
    const AABB2& b,
    Vec2& normal,
    float& depth
) noexcept
{
    if (!a.overlaps(b))
        return false;

    float ox = std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x);
    float oy = std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y);
    Vec2 diff = (a.min + a.max) - (b.min + b.max);

    if (ox < oy) {
        normal = { (diff.x < 0.0f) ? -1.0f : 1.0f, 0.0f };
        depth = ox;
    } else {
        normal = { 0.0f, (diff.y < 0.0f) ? -1.0f : 1.0f };
        depth = oy;
    }
    return true;
}

void
resolveAgainstStatic(
    Mover& m,
    const std::vector<AABB2>& static_boxes,
    float dt
) noexcept
{
    for (const AABB2& target : static_boxes) {
        Vec2 normal;
        float depth = 0.0f;
        const AABB2& mover_aabb = makeWorldAABB(*m.trans, *m.box);
        if (!computeOverlap(mover_aabb, target, normal, depth))
            continue;

        m.trans->pos += normal * depth;
        Vec2 v(m.vel->x, m.vel->y);
        float vn = v.dot(normal);
        if (vn < 0.0f) {
            v -= normal * vn;
            m.vel->x = v.x;
            m.vel->y = v.y;
        }
        if (normal.y < 0.0f && m.body)
            m.body->grounded = true;
    }

    for (int i = 0; i < 3; ++i) {
        AABB2 moving = makeWorldAABB(*m.trans, *m.box);
        Vec2 delta = Vec2(m.vel->x, m.vel->y) * dt;

        SweepHit earliest;
        for (const AABB2& target : static_boxes) {
            SweepHit hit = sweepAABB(moving, delta, target);
            if (hit.hit && hit.t < earliest.t)
                earliest = hit;
        }
        if (!earliest.hit)
            break;

        Vec2 v(m.vel->x, m.vel->y);
        float vn = v.dot(earliest.normal);
        if (vn >= 0.0f)
            break;

        if (earliest.normal.y < 0.0f && m.body)
            m.body->grounded = true;

        float bounce = m.body ? m.body->bounciness : 0.0f;
        if (bounce > 0.0f) {
            m.trans->pos += earliest.normal * (vn * dt * earliest.t);
            v -= earliest.normal * ((1.0f + bounce) * vn);
        } else
            v -= earliest.normal * (vn * (1.0f - earliest.t));

        if (m.body && m.body->contact_friction > 0.0f) {
            Vec2 tangent(-earliest.normal.y, earliest.normal.x);
            float damp = std::exp(-m.body->contact_friction * dt);
            v -= tangent * (v.dot(tangent) * (1.0f - damp));
        }

        m.vel->x = v.x;
        m.vel->y = v.y;
    }
}

void
applyPairImpulse(Mover& a, Mover& b, Vec2 normal, float scale) noexcept
{
    float ima = invMass(a.body);
    float imb = invMass(b.body);
    Vec2 rv(a.vel->x - b.vel->x, a.vel->y - b.vel->y);
    float vn = rv.dot(normal);
    if (vn >= 0.0f)
        return;

    float j = -scale * vn / (ima + imb);
    a.vel->x += normal.x * (j * ima);
    a.vel->y += normal.y * (j * ima);
    b.vel->x -= normal.x * (j * imb);
    b.vel->y -= normal.y * (j * imb);
}

void
resolveDynamicPair(Mover& a, Mover& b, float dt) noexcept
{
    AABB2 box_a = makeWorldAABB(*a.trans, *a.box);
    AABB2 box_b = makeWorldAABB(*b.trans, *b.box);

    float ima = invMass(a.body);
    float imb = invMass(b.body);
    float inv_sum = ima + imb;

    Vec2 normal;
    float depth = 0.0f;
    if (computeOverlap(box_a, box_b, normal, depth)) {
        a.trans->pos += normal * (depth * (ima / inv_sum));
        b.trans->pos -= normal * (depth * (imb / inv_sum));

        if (normal.y < 0.0f && a.body)
            a.body->grounded = true;
        if (normal.y > 0.0f && b.body)
            b.body->grounded = true;

        applyPairImpulse(a, b, normal, 1.0f);
        return;
    }

    Vec2 rv(a.vel->x - b.vel->x, a.vel->y - b.vel->y);
    SweepHit hit = sweepAABB(box_a, rv * dt, box_b);
    if (!hit.hit || rv.dot(hit.normal) >= 0.0f)
        return;

    if (hit.normal.y < 0.0f && a.body)
        a.body->grounded = true;
    if (hit.normal.y > 0.0f && b.body)
        b.body->grounded = true;

    float bounce = std::max(
        a.body ? a.body->bounciness : 0.0f, b.body ? b.body->bounciness : 0.0f
    );
    if (bounce > 0.0f) {
        Vec2 va(a.vel->x, a.vel->y);
        Vec2 vb(b.vel->x, b.vel->y);
        a.trans->pos += hit.normal * (va.dot(hit.normal) * dt * hit.t);
        b.trans->pos += hit.normal * (vb.dot(hit.normal) * dt * hit.t);
    }

    float scale = (bounce > 0.0f) ? (1.0f + bounce) : (1.0f - hit.t);
    applyPairImpulse(a, b, hit.normal, scale);

    float friction = std::max(
        a.body ? a.body->contact_friction : 0.0f,
        b.body ? b.body->contact_friction : 0.0f
    );
    if (friction > 0.0f) {
        Vec2 tangent(-hit.normal.y, hit.normal.x);
        float jt =
            -rv.dot(tangent) * (1.0f - std::exp(-friction * dt)) / inv_sum;
        a.vel->x += tangent.x * (jt * ima);
        a.vel->y += tangent.y * (jt * ima);
        b.vel->x -= tangent.x * (jt * imb);
        b.vel->y -= tangent.y * (jt * imb);
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
        [&](Entity e, const cTransform& trans, const cBoxCollider& box) {
            if (box.physical && !registry.hasComponent<cVelocity>(e))
                static_boxes.push_back(makeWorldAABB(trans, box));
        }
    );

    std::vector<Mover> movers;
    ECSView<cTransform, cVelocity, cBoxCollider>(registry).each(
        [&](
            Entity e, cTransform& trans, cVelocity& vel, const cBoxCollider& box
        ) {
            if (!box.physical)
                return;
            auto* body = registry.hasComponent<cDynamicBody>(e)
                             ? &registry.getComponent<cDynamicBody>(e)
                             : nullptr;
            movers.push_back({ &trans, &vel, &box, body });
        }
    );
    if (movers.empty())
        return;

    if (!static_boxes.empty())
        for (Mover& m : movers)
            resolveAgainstStatic(m, static_boxes, dt);

    for (size_t i = 0; i < movers.size(); ++i)
        for (size_t j = i + 1; j < movers.size(); ++j)
            resolveDynamicPair(movers[i], movers[j], dt);

    if (!static_boxes.empty())
        for (Mover& m : movers)
            resolveAgainstStatic(m, static_boxes, dt);
}

} // namespace lili
