#pragma once

#include "lili2d/components/dynamic_body.hpp"
#include "lili2d/components/velocity.hpp"
#include "lili2d/core/thread_pool.hpp"
#include "lili2d/ecs/ecs_registry.hpp"
#include "lili2d/ecs/system.hpp"
namespace lili {

class DynamicBodySystem : public System<cDynamicBody, cVelocity>
{
public:
    explicit DynamicBodySystem() noexcept {}

    void
    updateEntity(
        ECSRegistry& registry,
        Entity entity,
        lili::cDynamicBody& dyn_body_comp,
        lili::cVelocity& vel_comp,
        float dt
    ) override;

    static void
    fixedUpdate(ECSRegistry& registry, float dt, ThreadPool* thread_pool);
};

} // namespace lili
