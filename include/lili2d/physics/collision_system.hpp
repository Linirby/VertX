#pragma once

namespace lili {

class ECSRegistry;

class CollisionSystem
{
public:
    static void
    fixedUpdate(ECSRegistry& registry, float dt);
};

} // namespace lili
