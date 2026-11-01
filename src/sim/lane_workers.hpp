#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <thread>
#include <utility>
#include <vector>

#if defined(__APPLE__) && defined(__aarch64__)
#include <sys/sysctl.h>
#endif

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

// Bound runnable executors, not logical owners. On heterogeneous Apple hosts,
// prefer the high-performance physical-core budget; this is not CPU affinity.
inline std::size_t laneExecutionSlots() noexcept {
#if defined(__APPLE__) && defined(__aarch64__)
    unsigned cores = 0U;
    std::size_t size = sizeof(cores);
    if (sysctlbyname("hw.perflevel0.physicalcpu", &cores, &size, nullptr, 0U) == 0
        && size == sizeof(cores) && cores != 0U) return cores;
#endif
    return std::max(1U, std::thread::hardware_concurrency());
}

// Estimate the useful number of concurrent executors from measured per-owner
// work. Saturating/zero measurements deliberately choose the full available
// budget rather than serializing owners on incomplete timing information.
inline std::size_t laneActiveExecutionSlots(const std::vector<std::uint64_t>& costs,
                                           const std::size_t max_slots) noexcept {
    if (costs.empty() || max_slots == 0U) return 0U;
    std::uint64_t total = 0U;
    std::uint64_t largest = 0U;
    for (const auto cost : costs) {
        if (cost == 0U || total > UINT64_MAX - cost) return max_slots;
        total += cost;
        largest = std::max(largest, cost);
    }
    const auto slots = total / largest + (total % largest != 0U ? 1U : 0U);
    return std::min(max_slots, static_cast<std::size_t>(slots));
}

// Selects how long an idle executor spins before parking on the generation
// counter. A cross-board window is short, so parking and waking every window
// costs more wall time than the idle spin it saves, but only when the host has
// spare cores: with no slack the spin would compete with a working lane, so the
// conservative park-first budget is kept there.
inline std::size_t laneSpinBudget(
    const std::size_t executors, const std::size_t max_slots) noexcept {
    const auto slots = laneExecutionSlots();
    const auto used = std::min(executors, max_slots);
    return slots >= used + 2U ? 32U * 1024U : 64U;
}

// Persistent owner executors. Release/acquire publishes each window's active
// executor count. Jobs keep original owner IDs, but need not return to the same
// host thread. Every worker acknowledges every window before shared mutation.
// Only the coordinator calls run(); work and lane storage stay fixed until join.
class LaneWorkers {
public:
    LaneWorkers(std::size_t count, std::function<void(std::size_t)> work)
        : LaneWorkers(count, std::move(work), count == 0U ? 0U : count - 1U) {}

    LaneWorkers(std::size_t count, std::function<void(std::size_t)> work,
                std::size_t background_workers)
        : work_(std::move(work)), failures_(count), order_(count), costs_(count), count_(count),
          worker_count_(std::min(count, background_workers)),
          spin_limit_(laneSpinBudget(count, count == 0U ? 0U : count - 1U)) {
        for (std::size_t index = 0U; index < count_; ++index) order_[index] = index;
        threads_.reserve(worker_count_);
        try {
            for (std::size_t worker = 0U; worker < worker_count_; ++worker) {
                threads_.emplace_back([this, worker] {
                    ready_.wait(false, std::memory_order_acquire);
                    if (startup_failed_.load(std::memory_order_relaxed)) return;
                    std::uint64_t observed = 0U;
                    for (;;) {
                        std::size_t spins = 0U;
                        std::uint64_t next;
                        while ((next = generation_.load(std::memory_order_acquire)) == observed) {
                            if (spins++ < spin_limit_) laneSpinPause();
                            else generation_.wait(observed, std::memory_order_acquire);
                        }
                        observed = next;
                        if (stopping_.load(std::memory_order_relaxed)) return;
                        if (worker + 1U < execution_slots_) drain();
                        // Acknowledge only after the final queue access and all
                        // per-job scopes unwind. Last-job completion alone is
                        // insufficient: another consumer may still claim work.
                        if (remaining_.fetch_sub(1U, std::memory_order_acq_rel) == 1U)
                            remaining_.notify_one();
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
        generation_.notify_all();
        threads_.clear();
    }

    LaneWorkers(const LaneWorkers&) = delete;
    LaneWorkers& operator=(const LaneWorkers&) = delete;

    void run() {
        // Start expensive owners first. Costs are host-only scheduling hints;
        // every owner still executes exactly once and keeps its original ID.
        std::sort(order_.begin(), order_.end(), [&](auto a, auto b) {
            return costs_[a] != costs_[b] ? costs_[a] > costs_[b] : a < b;
        });
        execution_slots_ = laneActiveExecutionSlots(
            costs_, std::min(count_, worker_count_ + 1U));
        next_index_.store(0U, std::memory_order_relaxed);
        remaining_.store(worker_count_, std::memory_order_relaxed);
        generation_.fetch_add(1U, std::memory_order_release);
        generation_.notify_all();
        drain();
        std::size_t spins = 0U;
        for (auto remaining = remaining_.load(std::memory_order_acquire); remaining != 0U;
             remaining = remaining_.load(std::memory_order_acquire)) {
            if (spins++ < spin_limit_) laneSpinPause();
            else remaining_.wait(remaining, std::memory_order_acquire);
        }
        // Clear every slot before throwing, including simultaneous failures at
        // higher owner IDs. Host scheduling must not choose exception priority.
        std::exception_ptr first;
        for (auto& failure : failures_) {
            auto captured = std::exchange(failure, {});
            if (!first && captured) first = std::move(captured);
        }
        if (first) std::rethrow_exception(first);
    }

private:
    void drain() noexcept {
        for (;;) {
            const auto position = next_index_.fetch_add(1U, std::memory_order_relaxed);
            if (position >= count_) return;
            const auto index = order_[position];
            const auto start = std::chrono::steady_clock::now();
            try {
                work_(index);
            } catch (...) {
                failures_[index] = std::current_exception();
            }
            const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - start).count();
            const auto cost = elapsed > 0 ? static_cast<std::uint64_t>(elapsed) : 0U;
            costs_[index] = std::max(costs_[index] / 2U, cost);
        }
    }

    std::function<void(std::size_t)> work_;
    std::vector<std::exception_ptr> failures_;
    std::vector<std::size_t> order_;
    std::vector<std::uint64_t> costs_;
    std::size_t count_;
    std::size_t worker_count_;
    std::size_t spin_limit_;
    std::size_t execution_slots_ = 1U;
    alignas(64) std::atomic<std::uint64_t> generation_{0U};
    alignas(64) std::atomic<std::size_t> remaining_{0U};
    alignas(64) std::atomic<std::size_t> next_index_{0U};
    std::atomic<bool> stopping_{false};
    std::atomic<bool> ready_{false};
    std::atomic<bool> startup_failed_{false};
    std::vector<std::jthread> threads_;
};

} // namespace fil::sim::detail
