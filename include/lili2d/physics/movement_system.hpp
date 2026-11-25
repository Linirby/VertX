#pragma once

namespace lili {

class ECSRegistry;
class ThreadPool;

class MovementSystem
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
