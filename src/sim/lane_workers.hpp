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

/**
 * Monotonic host cycle counter. This is a pure register read used only for
 * host scheduling hints, so it can never observe guest state and never
 * allocates. `steady_clock::now()` on macOS funnels through
 * `clock_gettime_nsec_np`; at one sample per job per window that dominated the
 * coordinator profile.
 */
inline std::uint64_t laneCycleCount() noexcept {
#if defined(__aarch64__)
    std::uint64_t value;
    __asm__ __volatile__("mrs %0, cntvct_el0" : "=r"(value));
    return value;
#elif defined(__x86_64__) || defined(_M_X64)
    return static_cast<std::uint64_t>(__builtin_ia32_rdtsc());
#else
    return static_cast<std::uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
#endif
}

/** Cycle counts for one microsecond. Calibrated once against a steady clock. */
inline std::uint64_t laneCyclesPerMicrosecond() noexcept {
    static const std::uint64_t value = [] {
        const auto begin_time = std::chrono::steady_clock::now();
        const auto begin = laneCycleCount();
        while (std::chrono::steady_clock::now() - begin_time
               < std::chrono::microseconds{200}) {
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - begin_time).count();
        const auto cycles = laneCycleCount() - begin;
        if (elapsed <= 0 || cycles == 0U) return std::uint64_t{1000U};
        return (cycles / static_cast<std::uint64_t>(elapsed)) * 1000U;
    }();
    return value;
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
    const auto slots = std::max(laneExecutionSlots(),
        static_cast<std::size_t>(std::thread::hardware_concurrency()));
    const auto used = std::min(executors, max_slots);
    return slots >= used + 1U ? 32U * 1024U : 256U;
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
                        const auto spin_until = laneCycleCount() + spin_cycles_;
                        while (generation_.load(std::memory_order_acquire) == observed) {
                            if (laneCycleCount() < spin_until) {
                                laneSpinPause();
                                continue;
                            }
                            // Announce parking before the (atomically
                            // rechecked) wait so the coordinator can skip the
                            // futex syscall entirely while we still spin.
                            parked_.fetch_add(1U, std::memory_order_seq_cst);
                            generation_.wait(observed, std::memory_order_seq_cst);
                            parked_.fetch_sub(1U, std::memory_order_seq_cst);
                        }
                        const auto next = generation_.load(std::memory_order_acquire);
                        observed = next;
                        if (stopping_.load(std::memory_order_relaxed)) return;
                        if (!width_limited_ || worker + 1U < execution_slots_) drain();
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

    // Called only by the coordinator while every worker is parked. The
    // callback may borrow invocation-local state; release it on every exit.
    void run(std::function<void(std::size_t)> work) {
        work_ = std::move(work);
        struct ClearWork {
            std::function<void(std::size_t)>& work;
            ~ClearWork() { work = {}; }
        } clear{work_};
        run();
    }

    void run() {
        // Start expensive owners first. Costs are host-only scheduling hints;
        // every owner still executes exactly once and keeps its original ID.
        std::sort(order_.begin(), order_.end(), [&](auto a, auto b) {
            return costs_[a] != costs_[b] ? costs_[a] > costs_[b] : a < b;
        });
        // Every job must finish before the window can advance, so extra
        // executors can only shorten the critical path. Width limiting is
        // retained solely for callers that ask for it; the default keeps all
        // available executors draining the shared queue.
        execution_slots_ = width_limited_
            ? laneActiveExecutionSlots(costs_, std::min(count_, worker_count_ + 1U))
            : std::min(count_, worker_count_ + 1U);
        next_index_.store(0U, std::memory_order_relaxed);
        remaining_.store(worker_count_, std::memory_order_relaxed);
        const auto window_begin = laneCycleCount();
        generation_.fetch_add(1U, std::memory_order_release);
        if (parked_.load(std::memory_order_acquire) != 0U) generation_.notify_all();
        drain();
        std::size_t spins = 0U;
        const auto wait_until = window_begin + spin_cycles_;
        for (auto remaining = remaining_.load(std::memory_order_acquire); remaining != 0U;
             remaining = remaining_.load(std::memory_order_acquire)) {
            if (spins++ < spin_limit_ || laneCycleCount() < wait_until) {
                laneSpinPause();
                continue;
            }
            coordinator_parked_.fetch_add(1U, std::memory_order_seq_cst);
            remaining_.wait(remaining, std::memory_order_seq_cst);
            coordinator_parked_.fetch_sub(1U, std::memory_order_seq_cst);
        }
        observed_window_cycles_ = laneCycleCount() - window_begin;
        updateSpinCycles();
        // Clear every slot before throwing, including simultaneous failures at
        // higher owner IDs. Host scheduling must not choose exception priority.
        std::exception_ptr first;
        for (auto& failure : failures_) {
            auto captured = std::exchange(failure, {});
            if (!first && captured) first = std::move(captured);
        }
        if (first) std::rethrow_exception(first);
    }

    /**
     * @brief Limits active executors to the measured useful width.
     *
     * The shared queue makes extra executors strictly helpful for the window
     * barrier, so this is off by default and only used by callers that need
     * the historical measured-width policy.
     */
    void setWidthLimited(const bool limited) noexcept { width_limited_ = limited; }

private:
    // Spin for a fraction of the last observed window, so a parked executor
    // does not pay a futex round trip when the next window is already queued.
    void updateSpinCycles() noexcept {
        const auto per_microsecond = laneCyclesPerMicrosecond();
        const auto quarter = observed_window_cycles_ / 4U;
        const auto lower = per_microsecond * 2U;
        const auto upper = per_microsecond * 100U;
        spin_cycles_ = std::clamp(quarter, lower, upper);
    }

    void drain() noexcept {
        for (;;) {
            const auto position = next_index_.fetch_add(1U, std::memory_order_relaxed);
            if (position >= count_) return;
            const auto index = order_[position];
            const auto start = laneCycleCount();
            try {
                work_(index);
            } catch (...) {
                failures_[index] = std::current_exception();
            }
            const auto cost = laneCycleCount() - start;
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
    std::uint64_t spin_cycles_ = laneCyclesPerMicrosecond() * 20U;
    std::uint64_t observed_window_cycles_ = 0U;
    bool width_limited_ = false;
    alignas(64) std::atomic<std::uint64_t> generation_{0U};
    alignas(64) std::atomic<std::size_t> remaining_{0U};
    alignas(64) std::atomic<std::size_t> next_index_{0U};
    alignas(64) std::atomic<std::size_t> parked_{0U};
    alignas(64) std::atomic<std::size_t> coordinator_parked_{0U};
    std::atomic<bool> stopping_{false};
    std::atomic<bool> ready_{false};
    std::atomic<bool> startup_failed_{false};
    std::vector<std::jthread> threads_;
};

} // namespace fil::sim::detail
