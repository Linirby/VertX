#include "lili2d/physics/friction_system.hpp"

#include "lili2d/components/dynamic_body.hpp"
#include "lili2d/components/friction.hpp"
#include "lili2d/components/gravity.hpp"
#include "lili2d/components/velocity.hpp"
#include "lili2d/ecs/ecs_view.hpp"

#include <cmath>

namespace lili {

void
FrictionSystem::fixedUpdate(
    ECSRegistry& registry,
    float dt,
    ThreadPool* thread_pool
)
{
    (void)registry.getPool<cGravity>();
    (void)registry.getPool<cDynamicBody>();
    ECSView<cVelocity, cFriction>(registry).parallelEach(
        thread_pool,
        [&registry, dt](Entity entity, cVelocity& vel, cFriction& fric) {
            if (fric.amount <= 0.0f)
                return;

            if (registry.hasComponent<cDynamicBody>(entity)) {
                auto& body = registry.getComponent<cDynamicBody>(entity);
                if (!body.grounded)
                    return;
            }

            float damp = std::exp(-fric.amount * dt);
            vel.x *= damp;
            if (!registry.hasComponent<cGravity>(entity))
                vel.y *= damp;
        }
    );
}

} // namespace lili
