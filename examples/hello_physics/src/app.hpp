#pragma once

#include "lili2d/core/game.hpp"
#include <lili2d/ecs/entity.hpp>

class Game : public lili::Game
{
public:
    Game()
      : lili::Game("hello_physics - Lili2D", 800, 600)
    {
        this->getWindow()->setResizable(true);
    }

private:
    lili::Entity test_entity;

    void
    onInit() override;
    void
    onEvent(const lili::Event& event) override;
    void
    onUpdate(float dt) override;
    void
    onFixedUpdate(float dt) override;
};
