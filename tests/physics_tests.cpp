#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "lili2d/components/box_collider.hpp"
#include "lili2d/components/dynamic_body.hpp"
#include "lili2d/components/transform.hpp"
#include "lili2d/components/velocity.hpp"
#include "lili2d/ecs/ecs_registry.hpp"
#include "lili2d/physics.hpp"

using namespace lili;

TEST_CASE("AABB2 Intersection and Containment", "[physics][aabb]")
{
    AABB2 box1(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f));
    AABB2 box2(Vec2(5.0f, 5.0f), Vec2(10.0f, 10.0f));
    AABB2 box3(Vec2(20.0f, 20.0f), Vec2(10.0f, 10.0f));
    AABB2 inner(Vec2(2.0f, 2.0f), Vec2(4.0f, 4.0f));

    SECTION("Intersect")
    {
        CHECK(box1.intersect(box2));
        CHECK(box2.intersect(box1));
        CHECK_FALSE(box1.intersect(box3));
        CHECK_FALSE(box3.intersect(box1));
    }

    SECTION("Contains")
    {
        CHECK(box1.contains(inner));
        CHECK_FALSE(inner.contains(box1));
        CHECK_FALSE(box1.contains(box2));
    }

    SECTION("GetShape")
    {
        RectShape shape = box1.getShape();
        CHECK(shape.pos == Vec2(0.0f, 0.0f));
        CHECK(shape.size == Vec2(10.0f, 10.0f));

        AABB2 from_shape(shape);
        CHECK(from_shape.min == box1.min);
        CHECK(from_shape.max == box1.max);
    }

    SECTION("RectShape Interoperability")
    {
        RectShape rect_intersect(Vec2(5.0f, 5.0f), Vec2(10.0f, 10.0f));
        RectShape rect_miss(Vec2(20.0f, 20.0f), Vec2(10.0f, 10.0f));
        RectShape rect_contained(Vec2(2.0f, 2.0f), Vec2(4.0f, 4.0f));

        CHECK(box1.intersect(rect_intersect));
        CHECK_FALSE(box1.intersect(rect_miss));
        CHECK(box1.contains(rect_contained));
        CHECK_FALSE(box1.contains(rect_intersect));
    }

    SECTION("Overlaps (Non-Inclusive)")
    {
        AABB2 touching_edge(Vec2(10.0f, 0.0f), Vec2(10.0f, 10.0f));
        AABB2 touching_corner(Vec2(10.0f, 10.0f), Vec2(10.0f, 10.0f));

        CHECK(box1.overlaps(box2));
        CHECK(box2.overlaps(box1));
        CHECK_FALSE(box1.overlaps(box3));

        // Touching edge/corner: intersect is true, overlaps is false
        CHECK(box1.intersect(touching_edge));
        CHECK_FALSE(box1.overlaps(touching_edge));

        CHECK(box1.intersect(touching_corner));
        CHECK_FALSE(box1.overlaps(touching_corner));

        RectShape rect_touching(Vec2(10.0f, 0.0f), Vec2(5.0f, 5.0f));
        CHECK(box1.intersect(rect_touching));
        CHECK_FALSE(box1.overlaps(rect_touching));

        RectShape rect_overlapping(Vec2(5.0f, 5.0f), Vec2(5.0f, 5.0f));
        CHECK(box1.overlaps(rect_overlapping));
    }

    SECTION("LineShape Intersection and Overlaps")
    {
        LineShape line_thru(Vec2(-5.0f, 5.0f), Vec2(15.0f, 5.0f));
        LineShape line_inside(Vec2(2.0f, 2.0f), Vec2(8.0f, 8.0f));
        LineShape line_touching(Vec2(10.0f, -5.0f), Vec2(10.0f, 15.0f));
        LineShape line_corner(Vec2(10.0f, 10.0f), Vec2(15.0f, 15.0f));
        LineShape line_miss(Vec2(12.0f, 0.0f), Vec2(12.0f, 10.0f));
        LineShape point_touching(Vec2(10.0f, 5.0f), Vec2(10.0f, 5.0f));
        LineShape point_inside(Vec2(5.0f, 5.0f), Vec2(5.0f, 5.0f));

        CHECK(box1.intersect(line_thru));
        CHECK(box1.overlaps(line_thru));

        CHECK(box1.intersect(line_inside));
        CHECK(box1.overlaps(line_inside));

        CHECK(box1.intersect(line_touching));
        CHECK_FALSE(box1.overlaps(line_touching));

        CHECK(box1.intersect(line_corner));
        CHECK_FALSE(box1.overlaps(line_corner));

        CHECK_FALSE(box1.intersect(line_miss));
        CHECK_FALSE(box1.overlaps(line_miss));

        CHECK(box1.intersect(point_touching));
        CHECK_FALSE(box1.overlaps(point_touching));

        CHECK(box1.intersect(point_inside));
        CHECK(box1.overlaps(point_inside));
    }

    SECTION("Point Containment, Intersection and Overlaps")
    {
        CHECK(box1.contains(Vec2(5.0f, 5.0f)));
        CHECK(box1.intersect(Vec2(5.0f, 5.0f)));
        CHECK(box1.overlaps(Vec2(5.0f, 5.0f)));

        CHECK(box1.contains(Vec2(10.0f, 5.0f)));
        CHECK(box1.intersect(Vec2(10.0f, 5.0f)));
        CHECK_FALSE(box1.overlaps(Vec2(10.0f, 5.0f)));

        CHECK_FALSE(box1.contains(Vec2(15.0f, 5.0f)));
        CHECK_FALSE(box1.intersect(Vec2(15.0f, 5.0f)));
        CHECK_FALSE(box1.overlaps(Vec2(15.0f, 5.0f)));
    }
}

TEST_CASE("CircleCollider Intersection and Containment", "[physics][circle]")
{
    CircleCollider c1(Vec2(0.0f, 0.0f), 5.0f);
    CircleCollider c2(Vec2(6.0f, 0.0f), 3.0f);
    CircleCollider c3(Vec2(20.0f, 0.0f), 2.0f);
    CircleCollider inner(Vec2(1.0f, 0.0f), 2.0f);

    SECTION("Circle vs Circle Intersection and Overlaps")
    {
        CHECK(c1.intersect(c2));
        CHECK(c2.intersect(c1));
        CHECK(c1.overlaps(c2));
        CHECK(c2.overlaps(c1));

        CHECK_FALSE(c1.intersect(c3));
        CHECK_FALSE(c3.intersect(c1));
        CHECK_FALSE(c1.overlaps(c3));
        CHECK_FALSE(c3.overlaps(c1));

        // Tangent circles: intersect is true, overlaps is false
        CircleCollider c_tangent(Vec2(8.0f, 0.0f), 3.0f);
        CHECK(c1.intersect(c_tangent));
        CHECK(c_tangent.intersect(c1));
        CHECK_FALSE(c1.overlaps(c_tangent));
        CHECK_FALSE(c_tangent.overlaps(c1));
    }

    SECTION("Point Containment, Intersection and Overlaps")
    {
        CHECK(c1.contains(Vec2(0.0f, 0.0f)));
        CHECK(c1.intersect(Vec2(0.0f, 0.0f)));
        CHECK(c1.overlaps(Vec2(0.0f, 0.0f)));

        CHECK(c1.contains(Vec2(3.0f, 3.0f)));
        CHECK(c1.intersect(Vec2(3.0f, 3.0f)));
        CHECK(c1.overlaps(Vec2(3.0f, 3.0f)));

        // On boundary
        CHECK(c1.contains(Vec2(5.0f, 0.0f)));
        CHECK(c1.intersect(Vec2(5.0f, 0.0f)));
        CHECK_FALSE(c1.overlaps(Vec2(5.0f, 0.0f)));

        // Outside
        CHECK_FALSE(c1.contains(Vec2(5.1f, 0.0f)));
        CHECK_FALSE(c1.intersect(Vec2(5.1f, 0.0f)));
        CHECK_FALSE(c1.overlaps(Vec2(5.1f, 0.0f)));
    }

    SECTION("Circle vs Circle Containment")
    {
        CHECK(c1.contains(inner));
        CHECK_FALSE(inner.contains(c1));
        CHECK_FALSE(c1.contains(c2));

        CircleShape inner_shape(Vec2(1.0f, 0.0f), 2.0f);
        CHECK(c1.contains(inner_shape));
    }

    SECTION("Circle vs AABB2 Intersection and Overlaps")
    {
        AABB2 box(Vec2(3.0f, -2.0f), Vec2(4.0f, 4.0f));
        CHECK(c1.intersect(box));
        CHECK(c1.overlaps(box));
        CHECK(box.intersect(c1));
        CHECK(box.overlaps(c1));

        AABB2 far_box(Vec2(10.0f, 10.0f), Vec2(2.0f, 2.0f));
        CHECK_FALSE(c1.intersect(far_box));
        CHECK_FALSE(c1.overlaps(far_box));
        CHECK_FALSE(far_box.intersect(c1));
        CHECK_FALSE(far_box.overlaps(c1));

        // Tangent circle to box: touches at (5, 0)
        AABB2 tangent_box(Vec2(5.0f, -2.0f), Vec2(4.0f, 4.0f));
        CHECK(c1.intersect(tangent_box));
        CHECK_FALSE(c1.overlaps(tangent_box));
        CHECK(tangent_box.intersect(c1));
        CHECK_FALSE(tangent_box.overlaps(c1));
    }

    SECTION("Circle vs LineShape Intersection and Overlaps")
    {
        LineShape line_thru(Vec2(-10.0f, 0.0f), Vec2(10.0f, 0.0f));
        LineShape line_tangent(Vec2(-10.0f, 5.0f), Vec2(10.0f, 5.0f));
        LineShape line_miss(Vec2(-10.0f, 6.0f), Vec2(10.0f, 6.0f));
        LineShape point_tangent(Vec2(5.0f, 0.0f), Vec2(5.0f, 0.0f));
        LineShape point_inside(Vec2(2.0f, 0.0f), Vec2(2.0f, 0.0f));

        CHECK(c1.intersect(line_thru));
        CHECK(c1.overlaps(line_thru));

        CHECK(c1.intersect(line_tangent));
        CHECK_FALSE(c1.overlaps(line_tangent));

        CHECK_FALSE(c1.intersect(line_miss));
        CHECK_FALSE(c1.overlaps(line_miss));

        CHECK(c1.intersect(point_tangent));
        CHECK_FALSE(c1.overlaps(point_tangent));

        CHECK(c1.intersect(point_inside));
        CHECK(c1.overlaps(point_inside));
    }

    SECTION("GetShape")
    {
        CircleShape shape = c1.getShape();
        CHECK(shape.center == Vec2(0.0f, 0.0f));
        CHECK(shape.radius == 5.0f);
        CHECK(shape.segments == 16);

        CircleShape shape32 = c1.getShape(32);
        CHECK(shape32.segments == 32);

        CircleCollider from_shape(shape);
        CHECK(from_shape.center == c1.center);
        CHECK(from_shape.radius == c1.radius);
    }

    SECTION("RectShape Interoperability")
    {
        RectShape rect_hit(Vec2(3.0f, -2.0f), Vec2(4.0f, 4.0f));
        RectShape rect_miss(Vec2(10.0f, 10.0f), Vec2(2.0f, 2.0f));
        RectShape rect_tangent(Vec2(5.0f, -2.0f), Vec2(4.0f, 4.0f));

        CHECK(c1.intersect(rect_hit));
        CHECK(c1.overlaps(rect_hit));
        CHECK_FALSE(c1.intersect(rect_miss));
        CHECK_FALSE(c1.overlaps(rect_miss));

        CHECK(c1.intersect(rect_tangent));
        CHECK_FALSE(c1.overlaps(rect_tangent));

        CircleCollider from_rect(
            RectShape(Vec2(0.0f, 0.0f), Vec2(10.0f, 10.0f))
        );
        CHECK(from_rect.center == Vec2(5.0f, 5.0f));
        CHECK(from_rect.radius == 5.0f);
    }

    SECTION("CircleShape Interoperability")
    {
        CircleShape cs_overlap(Vec2(6.0f, 0.0f), 3.0f);
        CircleShape cs_tangent(Vec2(8.0f, 0.0f), 3.0f);
        CircleShape cs_miss(Vec2(20.0f, 0.0f), 2.0f);

        CHECK(c1.intersect(cs_overlap));
        CHECK(c1.overlaps(cs_overlap));

        CHECK(c1.intersect(cs_tangent));
        CHECK_FALSE(c1.overlaps(cs_tangent));

        CHECK_FALSE(c1.intersect(cs_miss));
        CHECK_FALSE(c1.overlaps(cs_miss));
    }
}

TEST_CASE("AABB3 Intersection, Overlaps, and Containment", "[physics][aabb3]")
{
    AABB3 box1(Vec3(0.0f, 0.0f, 0.0f), Vec3(10.0f, 10.0f, 10.0f));
    AABB3 box2(Vec3(5.0f, 5.0f, 5.0f), Vec3(10.0f, 10.0f, 10.0f));
    AABB3 touching(Vec3(10.0f, 0.0f, 0.0f), Vec3(10.0f, 10.0f, 10.0f));
    AABB3 miss(Vec3(20.0f, 20.0f, 20.0f), Vec3(10.0f, 10.0f, 10.0f));

    SECTION("AABB3 vs AABB3")
    {
        CHECK(box1.intersect(box2));
        CHECK(box1.overlaps(box2));

        CHECK(box1.intersect(touching));
        CHECK_FALSE(box1.overlaps(touching));

        CHECK_FALSE(box1.intersect(miss));
        CHECK_FALSE(box1.overlaps(miss));
    }

    SECTION("AABB3 Point Tests")
    {
        Vec3 inside(5.0f, 5.0f, 5.0f);
        Vec3 on_face(10.0f, 5.0f, 5.0f);
        Vec3 outside(15.0f, 5.0f, 5.0f);

        CHECK(box1.contains(inside));
        CHECK(box1.intersect(inside));
        CHECK(box1.overlaps(inside));

        CHECK(box1.contains(on_face));
        CHECK(box1.intersect(on_face));
        CHECK_FALSE(box1.overlaps(on_face));

        CHECK_FALSE(box1.contains(outside));
        CHECK_FALSE(box1.intersect(outside));
        CHECK_FALSE(box1.overlaps(outside));
    }
}

TEST_CASE("CollisionSystem Dynamic vs Static", "[physics][collision]")
{
    ECSRegistry registry;
    constexpr float dt = 0.1f;

    // Static floor: centered at (0, 100), size (200, 20) -> top surface at y =
    // 90
    Entity floor = registry.createEntity();
    registry.addComponents(
        floor,
        cTransform{ .pos = { 0.0f, 100.0f } },
        cBoxCollider{ .offset = { 0.0f, 0.0f }, .size = { 200.0f, 20.0f } }
    );

    SECTION("Landing on floor stops flush and sets grounded")
    {
        // Player 20x20 centered at (0, 75) -> bottom at y = 85 (5 px above
        // floor)
        Entity player = registry.createEntity();
        registry.addComponents(
            player,
            cTransform{ .pos = { 0.0f, 75.0f } },
            cVelocity{ .x = 0.0f,
                       .y = 100.0f }, // delta.y = 10 px -> hits at t = 0.5
            cBoxCollider{ .offset = { 0.0f, 0.0f }, .size = { 20.0f, 20.0f } },
            cDynamicBody{}
        );

        CollisionSystem::fixedUpdate(registry, dt);
        MovementSystem::fixedUpdate(registry, dt, nullptr);

        const auto& trans = registry.getComponent<cTransform>(player);
        const auto& body = registry.getComponent<cDynamicBody>(player);

        // Bottom should sit flush on y = 90 -> center at y = 80
        CHECK(trans.pos.y == Catch::Approx(80.0f));
        CHECK(body.grounded);
    }

    SECTION("High-speed projectile does not tunnel through thin wall")
    {
        // Thin wall: centered at (50, 0), size (4, 100) -> left face at x = 48
        Entity wall = registry.createEntity();
        registry.addComponents(
            wall,
            cTransform{ .pos = { 50.0f, 0.0f } },
            cBoxCollider{ .offset = { 0.0f, 0.0f }, .size = { 4.0f, 100.0f } }
        );

        // Fast 10x10 box at x = 0 moving +2000 px/s (delta.x = 200 px!)
        Entity bullet = registry.createEntity();
        registry.addComponents(
            bullet,
            cTransform{ .pos = { 0.0f, 0.0f } },
            cVelocity{ .x = 2000.0f, .y = 0.0f },
            cBoxCollider{ .offset = { 0.0f, 0.0f }, .size = { 10.0f, 10.0f } }
        );

        CollisionSystem::fixedUpdate(registry, dt);
        MovementSystem::fixedUpdate(registry, dt, nullptr);

        const auto& trans = registry.getComponent<cTransform>(bullet);
        // Right face touches wall left face (x = 48) -> center at x = 43
        CHECK(trans.pos.x == Catch::Approx(43.0f));
    }

    SECTION("Diagonal movement into floor+wall corner resolves both axes")
    {
        // Right wall: centered at (60, 0), size (20, 200) -> left face at x =
        // 50
        Entity wall = registry.createEntity();
        registry.addComponents(
            wall,
            cTransform{ .pos = { 60.0f, 0.0f } },
            cBoxCollider{ .offset = { 0.0f, 0.0f }, .size = { 20.0f, 200.0f } }
        );

        // 20x20 box at (32, 75): 8 px from wall, 5 px from floor
        Entity player = registry.createEntity();
        registry.addComponents(
            player,
            cTransform{ .pos = { 32.0f, 75.0f } },
            cVelocity{ .x = 100.0f, .y = 100.0f }, // delta = (10, 10)
            cBoxCollider{ .offset = { 0.0f, 0.0f }, .size = { 20.0f, 20.0f } },
            cDynamicBody{}
        );

        CollisionSystem::fixedUpdate(registry, dt);
        MovementSystem::fixedUpdate(registry, dt, nullptr);

        const auto& trans = registry.getComponent<cTransform>(player);
        const auto& body = registry.getComponent<cDynamicBody>(player);

        CHECK(trans.pos.x == Catch::Approx(40.0f));
        CHECK(trans.pos.y == Catch::Approx(80.0f));
        CHECK(body.grounded);
    }

    SECTION("Overlap fallback pushes out already-overlapping box")
    {
        // 20x20 box at (0, 84) -> bottom at 94 (4 px inside floor top at 90)
        Entity player = registry.createEntity();
        registry.addComponents(
            player,
            cTransform{ .pos = { 0.0f, 84.0f } },
            cVelocity{ .x = 0.0f, .y = 50.0f },
            cBoxCollider{ .offset = { 0.0f, 0.0f }, .size = { 20.0f, 20.0f } },
            cDynamicBody{}
        );

        CollisionSystem::fixedUpdate(registry, dt);
        MovementSystem::fixedUpdate(registry, dt, nullptr);

        const auto& trans = registry.getComponent<cTransform>(player);
        const auto& body = registry.getComponent<cDynamicBody>(player);

        CHECK(trans.pos.y == Catch::Approx(80.0f));
        CHECK(body.grounded);
    }

    SECTION("Bounciness reflects velocity off floor")
    {
        Entity ball = registry.createEntity();
        registry.addComponents(
            ball,
            cTransform{ .pos = { 0.0f, 75.0f } },
            cVelocity{ .x = 0.0f, .y = 100.0f },
            cBoxCollider{ .offset = { 0.0f, 0.0f }, .size = { 20.0f, 20.0f } },
            cDynamicBody{ .bounciness = 0.5f }
        );

        CollisionSystem::fixedUpdate(registry, dt);

        const auto& vel = registry.getComponent<cVelocity>(ball);
        CHECK(vel.y == Catch::Approx(-50.0f));
    }

    SECTION("Non-physical trigger does not block movement")
    {
        Entity trigger = registry.createEntity();
        registry.addComponents(
            trigger,
            cTransform{ .pos = { 20.0f, 0.0f } },
            cBoxCollider{
                .offset = { 0.0f, 0.0f },
                .size = { 10.0f, 100.0f },
                .physical = false,
            }
        );

        Entity mover = registry.createEntity();
        registry.addComponents(
            mover,
            cTransform{ .pos = { 0.0f, 0.0f } },
            cVelocity{ .x = 500.0f, .y = 0.0f },
            cBoxCollider{ .offset = { 0.0f, 0.0f }, .size = { 10.0f, 10.0f } }
        );

        CollisionSystem::fixedUpdate(registry, dt);
        MovementSystem::fixedUpdate(registry, dt, nullptr);

        const auto& trans = registry.getComponent<cTransform>(mover);
        CHECK(trans.pos.x == Catch::Approx(50.0f));
    }
}
