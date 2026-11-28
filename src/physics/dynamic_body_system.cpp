#include "lili2d/physics/dynamic_body_system.hpp"

#include "lili2d/components/dynamic_body.hpp"
#include "lili2d/components/velocity.hpp"
#include "lili2d/ecs/ecs_view.hpp"

#include <cmath>

namespace lili {

void
DynamicBodySystem::fixedUpdate(
    ECSRegistry& registry,
    float dt,
    ThreadPool* thread_pool
)
{
    ECSView<cDynamicBody, cVelocity>(registry).parallelEach(
        thread_pool, [dt](Entity, cDynamicBody& body, cVelocity& vel) {
            vel.x += body.force.x * dt / body.mass;
            vel.y += body.force.y * dt / body.mass;

            vel.x += body.impulse.x / body.mass;
            vel.y += body.impulse.y / body.mass;
            body.impulse = {};

            if (body.damping.x > 0.0f)
                vel.x *= std::exp(-body.damping.x * dt);
            if (body.damping.y > 0.0f)
                vel.y *= std::exp(-body.damping.y * dt);
        }
    );
}

} // namespace lili
