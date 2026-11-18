#include "lili2d/physics/dynamic_body_system.hpp"
#include <algorithm>

namespace lili {

void
DynamicBodySystem::updateEntity(
    ECSRegistry& registry,
    Entity entity,
    lili::cDynamicBody& dyn_body_comp,
    lili::cVelocity& vel_comp,
    float dt
)
{
    // TODO: check if the algo is working
    vel_comp.x += dyn_body_comp.force.x;
    vel_comp.y += dyn_body_comp.force.y;

    float slow_force = 0.0f;
    if (dyn_body_comp.grounded)
        slow_force =
            std::max(dyn_body_comp.mass * dyn_body_comp.contact_friction, 0.0f);
    else
        slow_force =
            std::max(dyn_body_comp.mass * dyn_body_comp.resistance, 0.0f);
    if (dyn_body_comp.force.x > 0 && slow_force != 0.0f)
        dyn_body_comp.force.x -= slow_force * dt;
    else if (dyn_body_comp.force.x < 0 && slow_force != 0.0f)
        dyn_body_comp.force.x += slow_force * dt;

    vel_comp.x += dyn_body_comp.impulse.x;
    vel_comp.y += dyn_body_comp.impulse.y;
    dyn_body_comp.impulse = {};
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
