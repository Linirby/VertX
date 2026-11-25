#include "app.hpp"

#include <lili2d/components/dynamic_body.hpp>
#include <lili2d/components/friction.hpp>
#include <lili2d/components/gravity.hpp>
#include <lili2d/components/transform.hpp>
#include <lili2d/components/velocity.hpp>
#include <lili2d/core/action_map.hpp>
#include <lili2d/core/event.hpp>
#include <lili2d/core/game.hpp>
#include <lili2d/core/keys.hpp>
#include <lili2d/core/mouse.hpp>
#include <lili2d/ecs/ecs_registry.hpp>
#include <lili2d/ecs/entity.hpp>
#include <lili2d/entities/rect.hpp>
#include <lili2d/geometry/vec2.hpp>
#include <lili2d/physics/dynamic_body_system.hpp>
#include <lili2d/physics/friction_system.hpp>
#include <lili2d/physics/gravity_system.hpp>
#include <lili2d/physics/movement_system.hpp>

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
            .render2d = {
                .size = lili::Vec2(50, 50),
                .use_interpolation = true,
            },
        }
    );
    ecs_reg.addComponents(
        test_entity,
        lili::cVelocity{},
        lili::cFriction{},
        lili::cDynamicBody{ .grounded = false }
    );

    getRenderer()->setCamera(&camera);
}

void
Game::onEvent(const lili::Event& event)
{
    lili::Game::onEvent(event);

    if (event.type() == lili::EventType::KEYBOARD) {
        lili::KeyboardEvent kb_ev = event.keyboard();
        if (event.keyJustPressed(kb_ev) && kb_ev.key == lili::Key::ESCAPE)
            shutdown();
    }
}

void
Game::onUpdate(float dt)
{
    lili::Game::onUpdate(dt);

    auto& actions = lili::ActionMap::get();
    auto& body = getECSRegistry().getComponent<lili::cDynamicBody>(test_entity);

    mouse.update();
    lili::Vec2 dir = { 0.0f, 0.0f };
    if (actions.isJustPressed("impulse_drag"))
        start_impulse_pos = mouse.getPos();
    else if (actions.isJustReleased("impulse_drag")) {
        lili::Vec2 end_impulse_pos = mouse.getPos();
        dir = end_impulse_pos - start_impulse_pos;
        body.impulse = -dir.normalized() * dir.length() * 10.0f;
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
    body.force = { 0.0f, 0.0f };
    body.force =
        dir.normalized() * ((body.grounded) ? GROUND_ACCEL : AIR_ACCEL);
}

void
Game::onFixedUpdate(float dt)
{
    auto& ecs_reg = getECSRegistry();
    auto* thread_pool = getThreadPool();

    lili::GravitySystem::fixedUpdate(ecs_reg, dt, thread_pool);
    lili::DynamicBodySystem::fixedUpdate(ecs_reg, dt, thread_pool);
    lili::FrictionSystem::fixedUpdate(ecs_reg, dt, thread_pool);
    lili::MovementSystem::fixedUpdate(ecs_reg, dt, thread_pool);
}
