#include "lili2d/physics/friction_system.hpp"
#include "lili2d/components/dynamic_body.hpp"
#include "lili2d/components/gravity.hpp"
#include <cmath>

namespace lili {

void
FrictionSystem::updateEntity(
    ECSRegistry& registry,
    Entity entity,
    lili::cVelocity& vel_comp,
    lili::cFriction& fric_comp,
    float dt
)
{
    if (fric_comp.amount <= 0.0f)
        return;

    if (registry.hasComponent<cDynamicBody>(entity)) {
        auto& body_comp = registry.getComponent<cDynamicBody>(entity);
        if (!body_comp.grounded)
            return;
    }

    float damp = std::exp(-fric_comp.amount * dt);
    vel_comp.x *= damp;
    if (!registry.hasComponent<cGravity>(entity))
        vel_comp.y *= damp;
}

void
FrictionSystem::fixedUpdate(
    ECSRegistry& registry,
    float dt,
    ThreadPool* thread_pool
)
{
    (void)registry.getPool<cGravity>();
    (void)registry.getPool<cDynamicBody>();
    auto sys = FrictionSystem();
    sys.run(registry, dt, thread_pool);
}

} // namespace lili
