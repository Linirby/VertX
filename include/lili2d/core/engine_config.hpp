#pragma once

#include <SDL3/SDL_gpu.h>

#include <cstddef>
#include <cstdint>

#include "lili2d/core/thread_pool.hpp"

namespace lili {

/// @brief Settings for thread pool and rendering modes.
struct EngineConfig
{
    /// @brief Target performance profile.
    PerformanceProfile profile = PerformanceProfile::CORRECT;

    /// @brief Number of CPU core threads to leave unallocated for system
    /// processes.
    size_t threads_to_leave_free = 1;
    /// @brief Explicit override for total worker thread count (0 uses auto
    /// calculation).
    size_t thread_count_override = 0;
    /// @brief Maximum cap on worker thread count.
    size_t max_thread_cap = 8;

    /// @brief GPU present mode configuration.
    SDL_GPUPresentMode potato_present_mode = SDL_GPU_PRESENTMODE_VSYNC;

    /// @brief Maximum frames per second cap (0 = unlimited / uncapped).
    uint32_t max_fps = 0;
};

} // namespace lili
