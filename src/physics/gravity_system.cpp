#include "lili2d/physics/gravity_system.hpp"

#include "lili2d/components/dynamic_body.hpp"
#include "lili2d/components/gravity.hpp"
#include "lili2d/components/velocity.hpp"
#include "lili2d/ecs/ecs_registry.hpp"
#include "lili2d/ecs/ecs_view.hpp"
#include "lili2d/ecs/entity.hpp"

namespace lili {

void
GravitySystem::fixedUpdate(
    ECSRegistry& registry,
    float dt,
    ThreadPool* thread_pool
)
{
    (void)registry.getPool<cDynamicBody>();
    ECSView<cVelocity, cGravity>(registry).parallelEach(
        thread_pool,
        [&registry, dt](Entity entity, cVelocity& vel, cGravity& grav) {
            if (vel.y >= grav.max_fall_speed)
                return;

            float increment = grav.force * grav.scale;
            if (registry.hasComponent<cDynamicBody>(entity)) {
                auto& body = registry.getComponent<cDynamicBody>(entity);
                increment *= body.mass;
            }
            vel.y += increment * dt;
        }
    );
}

} // namespace lili
