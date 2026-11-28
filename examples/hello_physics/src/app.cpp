#include "app.hpp"

#include <lili2d/components/box_collider.hpp>
#include <lili2d/components/dynamic_body.hpp>
#include <lili2d/components/transform.hpp>
#include <lili2d/components/ui_layout.hpp>
#include <lili2d/components/velocity.hpp>
#include <lili2d/core/action_map.hpp>
#include <lili2d/core/event.hpp>
#include <lili2d/core/game.hpp>
#include <lili2d/core/keys.hpp>
#include <lili2d/core/mouse.hpp>
#include <lili2d/ecs/ecs_registry.hpp>
#include <lili2d/ecs/entity.hpp>
#include <lili2d/entities/rect.hpp>
#include <lili2d/entities/text.hpp>
#include <lili2d/geometry/vec2.hpp>
#include <lili2d/geometry/vec4.hpp>
#include <lili2d/physics/aabb_collider.hpp>
#include <lili2d/physics/collision_system.hpp>
#include <lili2d/physics/dynamic_body_system.hpp>
#include <lili2d/physics/movement_system.hpp>
#include <lili2d/render/gpu/pass_types.hpp>

void
Game::onInit()
{
    auto& actions = lili::ActionMap::get();
    actions.add(
        "impulse_drag",
        { lili::Key::SPACE },
        { lili::MouseButton::LEFT, lili::MouseButton::RIGHT }
    );
    actions.add("move_left", { lili::Key::LEFT, lili::Key::A });
    actions.add("move_right", { lili::Key::RIGHT, lili::Key::D });
    actions.add("move_up", { lili::Key::UP, lili::Key::W });
    actions.add("move_down", { lili::Key::DOWN, lili::Key::S });

    auto& ecs_reg = getECSRegistry();
    test_entity = lili::createRect(
        ecs_reg,
        {
            .transform = { .pos = lili::Vec2(-50.0f, 0.0f), },
            .render2d = {
                .size = lili::Vec2(50, 50),
                .use_interpolation = true,
            },
        }
    );
    ecs_reg.addComponents(
        test_entity,
        lili::cVelocity{},
        lili::cDynamicBody{},
        lili::cBoxCollider{ .size = lili::Vec2(50.0f, 50.0f) }
    );

    lili::Vec2 win_size = getWindow()->getSize();
    borders[0] = lili::createRect(
        ecs_reg,
        {
            .transform = { .pos = lili::Vec2(0.0f, win_size.y * 0.5f), },
            .render2d = { .size = lili::Vec2(win_size.x, 50.0f), },
        }
    );
    ecs_reg.addComponents(
        borders[0], lili::cBoxCollider{ .size = lili::Vec2(win_size.x, 50.0f) }
    );
    borders[1] = lili::createRect(
        ecs_reg,
        {
            .transform = { .pos = lili::Vec2(0.0f, win_size.y * -0.5f) },
            .render2d = { .size = lili::Vec2(win_size.x, 50.0f) },
        }
    );
    ecs_reg.addComponents(
        borders[1], lili::cBoxCollider{ .size = lili::Vec2(win_size.x, 50.0f) }
    );
    borders[2] = lili::createRect(
        ecs_reg,
        {
            .transform = { .pos = lili::Vec2(win_size.x * -0.5f, 0.0f), },
            .render2d = { .size = lili::Vec2(50.0f, win_size.y), },
        }
    );
    ecs_reg.addComponents(
        borders[2], lili::cBoxCollider{ .size = lili::Vec2(50.0f, win_size.y) }
    );
    borders[3] = lili::createRect(
        ecs_reg,
        {
            .transform = { .pos = lili::Vec2(win_size.x * 0.5f, 0.0f), },
            .render2d = {
                .size = lili::Vec2(50.0f, win_size.y),
            },
        }
    );
    ecs_reg.addComponents(
        borders[3], lili::cBoxCollider{ .size = lili::Vec2(50.0f, win_size.y) }
    );

    blue_box = lili::createRect(
        ecs_reg,
        {
            .transform = { .pos = lili::Vec2(50.0f, 0.0f), },
            .render2d = {
                .color = lili::Vec4(0.0f, 0.0f, 1.0f, 1.0f),
                .size = lili::Vec2(50.0f, 50.0f),
                .use_interpolation = true,
            },
        }
    );
    ecs_reg.addComponents(
        blue_box,
        lili::cVelocity{},
        lili::cDynamicBody{ .mass = 1.0f },
        lili::cBoxCollider{ .size = lili::Vec2(50.0f, 50.0f) }
    );

    intruction_text = lili::createText(
        ecs_reg,
        "LMB/RMB + DRAG: move blue\n"
        "WASD:           move white\n"
        "R:              reset",
        {
            .transform = {
                .pos = lili::Vec2(0.0f, -win_size.y * 0.5f + 80.0f),
                .scale = lili::Vec2(4.0f, 4.0f),
            },
            .render2d = {
                .color = lili::Vec4(1.0f, 1.0f, 1.0f, 0.5f),
                .layer = -1.0f,
            },
        }
    );

    getRenderer()->setCamera(&camera);
}

void
Game::onEvent(const lili::Event& event)
{
    lili::Game::onEvent(event);

    if (event.type() == lili::EventType::KEYBOARD) {
        lili::KeyboardEvent kb_ev = event.keyboard();
        if (event.keyJustPressed(kb_ev)) {
            if (kb_ev.key == lili::Key::ESCAPE)
                shutdown();
            if (kb_ev.key == lili::Key::R) {
                lili::ECSRegistry& ecs_reg = getECSRegistry();
                auto& body_pool = ecs_reg.getPool<lili::cDynamicBody>();
                for (auto& body : body_pool.getComponents()) {
                    body.force = {};
                    body.impulse = {};
                }

                auto& vel_pool = ecs_reg.getPool<lili::cVelocity>();
                for (auto& vel : vel_pool.getComponents())
                    vel = {};

                auto& test_trans =
                    ecs_reg.getComponent<lili::cTransform>(test_entity);
                test_trans.pos = { -50.0f, 0.0f };
                test_trans.prev_pos = test_trans.pos;
                auto& box_trans =
                    ecs_reg.getComponent<lili::cTransform>(blue_box);
                box_trans.pos = { 50.0f, 0.0f };
                box_trans.prev_pos = box_trans.pos;
            }
        }
    }
}

void
Game::onUpdate(float dt)
{
    lili::Game::onUpdate(dt);

    lili::ECSRegistry& ecs_reg = getECSRegistry();

    auto& actions = lili::ActionMap::get();
    auto& body = ecs_reg.getComponent<lili::cDynamicBody>(test_entity);
    auto& box_body = ecs_reg.getComponent<lili::cDynamicBody>(blue_box);

    mouse.update();
    lili::Vec2 dir(0.0f, 0.0f);
    if (actions.isJustPressed("impulse_drag"))
        start_impulse_pos = mouse.getPos();
    else if (actions.isJustReleased("impulse_drag")) {
        lili::Vec2 end_impulse_pos = mouse.getPos();
        dir = end_impulse_pos - start_impulse_pos;
        box_body.impulse = -dir.normalized() * dir.length() * 10.0f;
    } else {
        if (actions.isHeld("move_left"))
            dir.x += -1.0f;
        if (actions.isHeld("move_right"))
            dir.x += 1.0f;
        if (actions.isHeld("move_up"))
            dir.y += -1.0f;
        if (actions.isHeld("move_down"))
            dir.y += 1.0f;
    }
    body.force = dir.normalized() * ACCEL;
}

void
Game::onFixedUpdate(float dt)
{
    auto& ecs_reg = getECSRegistry();
    auto* thread_pool = getThreadPool();

    lili::DynamicBodySystem::fixedUpdate(ecs_reg, dt, thread_pool);
    lili::CollisionSystem::fixedUpdate(ecs_reg, dt);
    lili::MovementSystem::fixedUpdate(ecs_reg, dt, thread_pool);
}
