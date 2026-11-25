#pragma once

#include "lili2d/ecs/ecs_registry.hpp"

namespace lili {

class CollisionSystem
{
public:
    static void
    fixedUpdate(ECSRegistry& registry, float dt);
};

} // namespace lili
