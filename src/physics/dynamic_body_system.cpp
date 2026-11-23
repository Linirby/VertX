#include "lili2d/physics/dynamic_body_system.hpp"
#include <cmath>

namespace lili {

void
DynamicBodySystem::updateEntity(
    [[maybe_unused]] ECSRegistry& registry,
    [[maybe_unused]] Entity entity,
    lili::cDynamicBody& dyn_body_comp,
    lili::cVelocity& vel_comp,
    float dt
)
{
    vel_comp.x += dyn_body_comp.force.x;
    vel_comp.y += dyn_body_comp.force.y;

    vel_comp.x += dyn_body_comp.impulse.x;
    vel_comp.y += dyn_body_comp.impulse.y;
    dyn_body_comp.impulse = {};

    if (dyn_body_comp.air_drag > 0.0f && !dyn_body_comp.grounded) {
        float damp = std::exp(-dyn_body_comp.air_drag * dt);
        vel_comp.x *= damp;
        vel_comp.y *= damp;
    }
}

void
DynamicBodySystem::fixedUpdate(
    ECSRegistry& registry,
    float dt,
    ThreadPool* thread_pool
)
{
    auto sys = DynamicBodySystem();
    sys.run(registry, dt, thread_pool);
}

} // namespace lili
