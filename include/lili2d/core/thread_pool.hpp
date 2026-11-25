#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace lili {

struct EngineConfig;

/// @brief Performance profile options to scale threading and rendering
/// strategies.
enum class PerformanceProfile
{
    YES,     ///< Standard performance profile.
    CORRECT, ///< Balanced performance and resource efficiency profile.
    INSANE   ///< Maximum throughput high-performance profile.
};

/// @brief Task priority options for tasks in a thread pool
enum class TaskPriority
{
    HIGH,   ///< High priority task executed first.
    NORMAL, ///< Normal priority task.
    LOW     ///< Low priority background task.
};

/// @brief A simple, lightweight C++20 Thread Pool using jthread and
/// stop_token.
class ThreadPool
{
public:
    /// @brief Constructs a ThreadPool with default engine configuration.
    explicit ThreadPool();

    /// @brief Constructs a ThreadPool with specified engine configuration.
    /// @param config Engine configuration settings.
    explicit ThreadPool(const EngineConfig& config);

    /// @brief Deleted copy constructor.
    ThreadPool(const ThreadPool&) = delete;
    /// @brief Deleted copy assignment operator.
    ThreadPool&
    operator=(const ThreadPool&) = delete;
    /// @brief Deleted move constructor.
    ThreadPool(ThreadPool&&) = delete;
    /// @brief Deleted move assignment operator.
    ThreadPool&
    operator=(ThreadPool&&) = delete;

    /// @brief Enqueues a task for execution in the thread pool.
    /// @param task A void() function to execute.
    /// @param priority Priority level of the task (default:
    /// TaskPriority::NORMAL).
    void
    enqueue(
        std::function<void()> task,
        TaskPriority priority = TaskPriority::NORMAL
    );

    /// @brief Returns the active performance profile.
    /// @return Active PerformanceProfile.
    [[nodiscard]] inline PerformanceProfile
    getProfile() const noexcept
    {
        return profile;
    }

    /// @brief Returns the number of worker threads in the pool.
    /// @return Number of active worker threads.
    [[nodiscard]] inline size_t
    getThreadCount() const noexcept
    {
        return threads.size();
    }

private:
    static size_t
    calculateThreadCount(const EngineConfig& config);
    void
    worker_loop(std::stop_token stop_tok);

    std::queue<std::function<void()>> high_tasks;
    std::queue<std::function<void()>> normal_tasks;
    std::queue<std::function<void()>> low_tasks;
    std::mutex queue_mutex;
    std::condition_variable_any cv;
    std::vector<std::jthread> threads;
    PerformanceProfile profile;
};

} // namespace lili
