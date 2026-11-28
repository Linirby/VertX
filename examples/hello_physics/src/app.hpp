#pragma once

#include "lili2d/core/game.hpp"
#include <array>
#include <lili2d/core/mouse.hpp>
#include <lili2d/ecs/entity.hpp>
#include <lili2d/geometry/vec2.hpp>
#include <lili2d/render/common/camera.hpp>

class Game : public lili::Game
{
public:
    Game()
      : lili::Game("hello_physics - Lili2D", 800, 600)
    {
        this->getWindow()->setResizable(true);
    }

private:
    const float ACCEL = 75.0f;

    lili::Mouse mouse;
    lili::Vec2 start_impulse_pos = { 0.0f, 0.0f };

    lili::Entity test_entity;
    std::array<lili::Entity, 4> borders{};
    lili::Entity blue_box;

    lili::Camera camera;
    lili::Entity intruction_text;

    void
    onInit() override;
    void
    onEvent(const lili::Event& event) override;
    void
    onUpdate(float dt) override;
    void
    onFixedUpdate(float dt) override;
};
