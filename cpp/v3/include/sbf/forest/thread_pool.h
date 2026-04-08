// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — ThreadPool: lightweight C++17 thread pool
//  Module: sbf::forest
//
//  A minimal header-only thread pool for parallel forest growing.
//  Usage:
//      ThreadPool pool(4);
//      auto f = pool.submit([](int x) { return x * 2; }, 21);
//      int result = f.get();  // 42
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>

namespace sbf {
namespace forest {

class ThreadPool {
public:
    /// Create a thread pool with the given number of worker threads.
    explicit ThreadPool(int n_threads)
        : stop_(false)
    {
        for (int i = 0; i < n_threads; ++i) {
            workers_.emplace_back([this] {
                for (;;) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        cv_.wait(lock, [this] {
                            return stop_ || !tasks_.empty();
                        });
                        if (stop_ && tasks_.empty()) return;
                        task = std::move(tasks_.front());
                        tasks_.pop();
                    }
                    task();
                }
            });
        }
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& w : workers_)
            w.join();
    }

    // Non-copyable, non-moveable
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;

    /// Submit a callable + args for asynchronous execution.
    /// Returns a future for the result.
    template<typename F, typename... Args>
    auto submit(F&& f, Args&&... args)
        -> std::future<std::invoke_result_t<std::decay_t<F>, std::decay_t<Args>...>>
    {
        using return_type = std::invoke_result_t<std::decay_t<F>, std::decay_t<Args>...>;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

        std::future<return_type> result = task->get_future();
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (stop_)
                throw std::runtime_error("submit() on stopped ThreadPool");
            tasks_.emplace([task]() { (*task)(); });
        }
        cv_.notify_one();
        return result;
    }

    /// Number of worker threads.
    int size() const { return static_cast<int>(workers_.size()); }

private:
    std::vector<std::thread>          workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex                        mutex_;
    std::condition_variable           cv_;
    bool                              stop_;
};

} // namespace forest
} // namespace sbf
