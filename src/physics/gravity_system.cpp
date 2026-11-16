#include "lili2d/physics/gravity_system.hpp"
#include "lili2d/core/thread_pool.hpp"
#include "lili2d/ecs/ecs_registry.hpp"

namespace lili {

void
GravitySystem::updateEntity(
    [[maybe_unused]] ECSRegistry& registry,
    [[maybe_unused]] Entity entity,
    lili::cVelocity& vel_comp,
    lili::cGravity& grav_comp,
    float dt
)
{
    if (vel_comp.y >= grav_comp.max_fall_speed) {
        vel_comp.y = grav_comp.max_fall_speed;
        return;
    }
    vel_comp.y += grav_comp.force * grav_comp.scale * dt;
}

void
GravitySystem::fixedUpdate(
    ECSRegistry& registry,
    float dt,
    ThreadPool* thread_pool
)
{
    auto sys = GravitySystem();
    sys.run(registry, dt, thread_pool);
}

} // namespace lili
