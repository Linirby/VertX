#pragma once

#include "lili2d/components/friction.hpp"
#include "lili2d/components/velocity.hpp"
#include "lili2d/core/thread_pool.hpp"
#include "lili2d/ecs/ecs_registry.hpp"
#include "lili2d/ecs/system.hpp"
namespace lili {

class FrictionSystem : public System<cVelocity, cFriction>
{
public:
    static void
    fixedUpdate(ECSRegistry& registry, float dt, ThreadPool* thread_pool);

private:
    explicit FrictionSystem() noexcept {}

    void
    updateEntity(
        ECSRegistry& registry,
        Entity entity,
        lili::cVelocity& vel_comp,
        lili::cFriction& fric_comp,
        float dt
    ) override;
};

} // namespace lili
