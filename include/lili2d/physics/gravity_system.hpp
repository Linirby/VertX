#pragma once

#include "lili2d/components/gravity.hpp"
#include "lili2d/components/velocity.hpp"
#include "lili2d/core/thread_pool.hpp"
#include "lili2d/ecs/ecs_registry.hpp"
#include "lili2d/ecs/system.hpp"

namespace lili {

class GravitySystem : public System<cVelocity, cGravity>
{
public:
    explicit GravitySystem() noexcept {}

    void
    updateEntity(
        ECSRegistry& registry,
        Entity entity,
        lili::cVelocity& vel_comp,
        lili::cGravity& grav_comp,
        float dt
    ) override;

    static void
    fixedUpdate(ECSRegistry& registry, float dt, ThreadPool* thread_pool);
};

} // namespace lili
