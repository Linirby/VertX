#include "lili2d/physics/movement_system.hpp"

namespace lili {

void
MovementSystem::fixedUpdate(
    ECSRegistry& registry,
    float dt,
    ThreadPool* thread_pool
)
{
    auto sys = MovementSystem();
    sys.run(registry, dt, thread_pool);
}

void
MovementSystem::updateEntity(
    [[maybe_unused]] ECSRegistry& registry,
    [[maybe_unused]] Entity entity,
    lili::cVelocity& vel_comp,
    lili::cTransform& trans_comp,
    float dt
)
{
    trans_comp.prev_pos = trans_comp.pos;
    trans_comp.pos += lili::Vec2(vel_comp.x, vel_comp.y) * dt;
}

} // namespace lili
