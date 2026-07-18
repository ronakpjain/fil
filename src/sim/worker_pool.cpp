#include "fil/sim/worker_pool.hpp"

#include <atomic>
#include <exception>
#include <stdexcept>
#include <thread>
#include <vector>

namespace fil::sim {

struct LaneWorkerPool::Impl {
    explicit Impl(const std::size_t lane_count)
        : lanes(lane_count), failures(lane_count) {
        if (lanes == 0U) throw std::invalid_argument("worker pool requires at least one lane");
        workers.reserve(lanes);
        for (std::size_t lane = 0; lane < lanes; ++lane) {
            workers.emplace_back([this, lane] { worker(lane); });
        }
    }

    ~Impl() {
        stopping.store(true, std::memory_order_relaxed);
        generation.fetch_add(1U, std::memory_order_release);
        generation.notify_all();
        for (std::thread& worker_thread : workers) {
            if (worker_thread.joinable()) worker_thread.join();
        }
    }

    void worker(const std::size_t lane) {
        std::size_t observed_generation = 0U;
        for (;;) {
            generation.wait(observed_generation, std::memory_order_acquire);
            observed_generation = generation.load(std::memory_order_acquire);
            if (stopping.load(std::memory_order_relaxed)) return;
            try {
                task(lane);
            } catch (...) {
                failures[lane] = std::current_exception();
            }
            if (remaining.fetch_sub(1U, std::memory_order_release) == 1U) {
                remaining.notify_one();
            }
        }
    }

    void dispatch(const std::function<void(std::size_t)>& next_task) {
        if (remaining.load(std::memory_order_relaxed) != 0U) {
            throw std::logic_error("worker pool dispatch overlaps prior epoch");
        }
        task = next_task;
        for (std::exception_ptr& failure : failures) failure = nullptr;
        remaining.store(lanes, std::memory_order_relaxed);
        generation.fetch_add(1U, std::memory_order_release);
        generation.notify_all();

        std::size_t outstanding = remaining.load(std::memory_order_acquire);
        while (outstanding != 0U) {
            remaining.wait(outstanding, std::memory_order_acquire);
            outstanding = remaining.load(std::memory_order_acquire);
        }

        task = {};
        for (const std::exception_ptr& failure : failures) {
            if (failure) std::rethrow_exception(failure);
        }
    }

    std::size_t lanes{0U};
    std::vector<std::thread> workers;
    std::function<void(std::size_t)> task;
    std::vector<std::exception_ptr> failures;
    std::atomic<std::size_t> generation{0U};
    std::atomic<std::size_t> remaining{0U};
    std::atomic<bool> stopping{false};
};

LaneWorkerPool::LaneWorkerPool(const std::size_t lanes)
    : impl_(std::make_unique<Impl>(lanes)) {}

LaneWorkerPool::~LaneWorkerPool() = default;

void LaneWorkerPool::run(const std::function<void(std::size_t)>& task) {
    if (!task) throw std::invalid_argument("worker pool task is empty");
    impl_->dispatch(task);
}

std::size_t LaneWorkerPool::size() const noexcept {
    return impl_->lanes;
}

} // namespace fil::sim
