#pragma once

namespace lili {

class ECSRegistry;
class ThreadPool;

class FrictionSystem
{
public:
    static void
    fixedUpdate(
        ECSRegistry& registry,
        float dt,
        ThreadPool* thread_pool = nullptr
    );
};

} // namespace lili
