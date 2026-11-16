#include "lili2d/physics/dynamic_body_system.hpp"

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
    // TODO: update entity logic (forces + impulse add to velocity and reset)
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
