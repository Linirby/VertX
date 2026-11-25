#include "lili2d/physics/movement_system.hpp"

#include "lili2d/components/transform.hpp"
#include "lili2d/components/velocity.hpp"
#include "lili2d/ecs/ecs_view.hpp"
#include "lili2d/geometry/vec2.hpp"

namespace lili {

void
MovementSystem::fixedUpdate(
    ECSRegistry& registry,
    float dt,
    ThreadPool* thread_pool
)
{
    ECSView<cVelocity, cTransform>(registry).parallelEach(
        thread_pool,
        [dt](Entity, cVelocity& vel, cTransform& trans) {
            trans.prev_pos = trans.pos;
            trans.pos += Vec2(vel.x, vel.y) * dt;
        }
    );
}

} // namespace lili
