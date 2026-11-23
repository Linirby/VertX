#include "lili2d/physics/gravity_system.hpp"
#include "lili2d/components/dynamic_body.hpp"
#include "lili2d/core/thread_pool.hpp"
#include "lili2d/ecs/ecs_registry.hpp"
#include "lili2d/ecs/entity.hpp"

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
    if (vel_comp.y >= grav_comp.max_fall_speed)
        return;

    float increment = grav_comp.force * grav_comp.scale;
    if (registry.hasComponent<cDynamicBody>(entity)) {
        auto& body = registry.getComponent<cDynamicBody>(entity);
        increment *= body.mass;
    }
    vel_comp.y += increment * dt;
}

void
GravitySystem::fixedUpdate(
    ECSRegistry& registry,
    float dt,
    ThreadPool* thread_pool
)
{
    (void)registry.getPool<cDynamicBody>();
    auto sys = GravitySystem();
    sys.run(registry, dt, thread_pool);
}

} // namespace lili
