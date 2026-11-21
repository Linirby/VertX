#pragma once

#include "lili2d/components/transform.hpp"
#include "lili2d/components/velocity.hpp"
#include "lili2d/core/thread_pool.hpp"
#include "lili2d/ecs/ecs_registry.hpp"
#include "lili2d/ecs/system.hpp"
#include <lili2d/geometry/vec2.hpp>
namespace lili {

class MovementSystem : public System<cVelocity, cTransform>
{
public:
    static void
    fixedUpdate(ECSRegistry& registry, float dt, ThreadPool* thread_pool);

private:
    explicit MovementSystem() noexcept {}

    void
    updateEntity(
        [[maybe_unused]] ECSRegistry& registry,
        [[maybe_unused]] Entity entity,
        lili::cVelocity& vel_comp,
        lili::cTransform& trans_comp,
        float dt
    ) override;
};

} // namespace lili
