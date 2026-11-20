#include "app.hpp"
#include <lili2d/components/dynamic_body.hpp>
#include <lili2d/components/gravity.hpp>
#include <lili2d/components/transform.hpp>
#include <lili2d/components/velocity.hpp>
#include <lili2d/core/action_map.hpp>
#include <lili2d/core/event.hpp>
#include <lili2d/core/game.hpp>
#include <lili2d/core/keys.hpp>
#include <lili2d/ecs/ecs_registry.hpp>
#include <lili2d/ecs/entity.hpp>
#include <lili2d/entities/rect.hpp>
#include <lili2d/physics/dynamic_body_system.hpp>
#include <lili2d/physics/gravity_system.hpp>

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
                .size = { 50.0f, 50.0f },
            },
        }
    );
    ecs_reg.addComponents(
        test_entity,
        lili::cVelocity{},
        lili::cGravity{ .force = 0.0f },
        lili::cDynamicBody{ .grounded = true }
    );
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

    if (actions.isHeld("impulse_drag")) {
    } else {
        if (actions.isHeld("move_left"))
            body.force.x = -50.0f;
        if (actions.isHeld("move_right"))
            body.force.x = 50.0f;
        if (actions.isHeld("move_up"))
            body.force.y = -50.0f;
        if (actions.isHeld("move_down"))
            body.force.y = 50.0f;
    }
}

void
Game::onFixedUpdate(float dt)
{
    auto& ecs_reg = getECSRegistry();
    auto* thread_pool = getThreadPool();

    lili::GravitySystem::fixedUpdate(ecs_reg, dt, thread_pool);
    lili::DynamicBodySystem::fixedUpdate(ecs_reg, dt, thread_pool);

    auto& trans = ecs_reg.getComponent<lili::cTransform>(test_entity);
    auto& vel = ecs_reg.getComponent<lili::cVelocity>(test_entity);
    trans.pos.x += vel.x * dt;
    trans.pos.y += vel.y * dt;
}
