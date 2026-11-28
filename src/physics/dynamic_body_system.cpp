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
            vel.x += body.force.x;
            vel.y += body.force.y;

            vel.x += body.impulse.x;
            vel.y += body.impulse.y;
            body.impulse = {};

            if (body.air_drag > 0.0f && !body.grounded) {
                float damp = std::exp(-body.air_drag * dt);
                vel.x *= damp;
                vel.y *= damp;
            }
        }
    );
}

} // namespace lili
