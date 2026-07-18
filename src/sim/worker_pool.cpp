#include "fil/sim/worker_pool.hpp"

#include <condition_variable>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace fil::sim {

struct LaneWorkerPool::Impl {
    explicit Impl(const std::size_t lane_count) : lanes(lane_count) {
        if (lanes == 0U) throw std::invalid_argument("worker pool requires at least one lane");
        workers.reserve(lanes);
        for (std::size_t lane = 0; lane < lanes; ++lane) {
            workers.emplace_back([this, lane] { worker(lane); });
        }
    }

    ~Impl() {
        {
            std::lock_guard lock(mutex);
            stopping = true;
            ++generation;
        }
        start.notify_all();
        for (std::thread& worker_thread : workers) {
            if (worker_thread.joinable()) worker_thread.join();
        }
    }

    void worker(const std::size_t lane) {
        std::size_t observed_generation = 0U;
        for (;;) {
            std::function<void(std::size_t)> current;
            {
                std::unique_lock lock(mutex);
                start.wait(lock, [&] {
                    return stopping || generation != observed_generation;
                });
                if (stopping) return;
                observed_generation = generation;
                current = task;
            }

            try {
                current(lane);
            } catch (...) {
                std::lock_guard lock(mutex);
                if (!failure) failure = std::current_exception();
            }

            {
                std::lock_guard lock(mutex);
                if (--remaining == 0U) complete.notify_one();
            }
        }
    }

    void dispatch(const std::function<void(std::size_t)>& next_task) {
        {
            std::lock_guard lock(mutex);
            if (remaining != 0U) throw std::logic_error("worker pool dispatch overlaps prior epoch");
            task = next_task;
            failure = nullptr;
            remaining = lanes;
            ++generation;
        }
        start.notify_all();

        std::exception_ptr captured;
        {
            std::unique_lock lock(mutex);
            complete.wait(lock, [&] { return remaining == 0U; });
            captured = failure;
            task = {};
        }
        if (captured) std::rethrow_exception(captured);
    }

    std::size_t lanes{0U};
    std::vector<std::thread> workers;
    std::mutex mutex;
    std::condition_variable start;
    std::condition_variable complete;
    std::function<void(std::size_t)> task;
    std::exception_ptr failure;
    std::size_t generation{0U};
    std::size_t remaining{0U};
    bool stopping{false};
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
