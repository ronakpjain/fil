#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <thread>
#include <utility>
#include <vector>

namespace fil::sim::detail {

inline void laneSpinPause() noexcept {
#if defined(__i386__) || defined(__x86_64__)
    __builtin_ia32_pause();
#elif defined(__aarch64__)
    __asm__ __volatile__("yield");
#else
    std::atomic_signal_fence(std::memory_order_seq_cst);
#endif
}

// Persistent owner executors. The short window rendezvous uses release/acquire
// publication, not two kernel-backed barriers per simulated microsecond. Long
// waits yield periodically so startup/JIT compilation need not monopolize CPUs.
// Only the coordinator calls run(); work and lane storage stay fixed until join.
class LaneWorkers {
public:
    LaneWorkers(std::size_t count, std::function<void(std::size_t)> work)
        : work_(std::move(work)), failures_(count), count_(count) {
        threads_.reserve(count);
        try {
            for (std::size_t index = 0; index < count; ++index) {
                threads_.emplace_back([this, index] {
                    ready_.wait(false, std::memory_order_acquire);
                    if (startup_failed_.load(std::memory_order_relaxed)) return;
                    std::uint64_t observed = 0U;
                    for (;;) {
                        std::size_t spins = 0U;
                        std::uint64_t next;
                        while ((next = generation_.load(std::memory_order_acquire)) == observed) {
                            pause(spins);
                        }
                        observed = next;
                        if (stopping_.load(std::memory_order_relaxed)) return;
                        try {
                            work_(index);
                        } catch (...) {
                            failures_[index] = std::current_exception();
                        }
                        remaining_.fetch_sub(1U, std::memory_order_acq_rel);
                    }
                });
            }
        } catch (...) {
            startup_failed_.store(true, std::memory_order_relaxed);
            ready_.store(true, std::memory_order_release);
            ready_.notify_all();
            throw; // jthread members join the partially constructed pool.
        }
        ready_.store(true, std::memory_order_release);
        ready_.notify_all();
    }

    ~LaneWorkers() {
        stopping_.store(true, std::memory_order_relaxed);
        generation_.fetch_add(1U, std::memory_order_release);
        threads_.clear();
    }

    LaneWorkers(const LaneWorkers&) = delete;
    LaneWorkers& operator=(const LaneWorkers&) = delete;

    void run() {
        remaining_.store(count_, std::memory_order_relaxed);
        generation_.fetch_add(1U, std::memory_order_release);
        std::size_t spins = 0U;
        while (remaining_.load(std::memory_order_acquire) != 0U) pause(spins);
        for (auto& failure : failures_) {
            if (failure) {
                auto captured = std::exchange(failure, {});
                std::rethrow_exception(captured);
            }
        }
    }

private:
    static void pause(std::size_t& spins) noexcept {
        laneSpinPause();
        if (++spins == 4096U) {
            spins = 0U;
            std::this_thread::yield();
        }
    }

    std::function<void(std::size_t)> work_;
    std::vector<std::exception_ptr> failures_;
    std::size_t count_;
    alignas(64) std::atomic<std::uint64_t> generation_{0U};
    alignas(64) std::atomic<std::size_t> remaining_{0U};
    std::atomic<bool> stopping_{false};
    std::atomic<bool> ready_{false};
    std::atomic<bool> startup_failed_{false};
    std::vector<std::jthread> threads_;
};

} // namespace fil::sim::detail
