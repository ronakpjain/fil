#pragma once

/** @file worker_pool.hpp
 *  @brief Persistent fixed-lane worker threads for transactional board epochs.
 */

#include <cstddef>
#include <functional>
#include <memory>

namespace fil::sim {

/** @brief Reuses one host worker per board lane across scheduler epochs. */
class LaneWorkerPool {
public:
    explicit LaneWorkerPool(std::size_t lanes);
    ~LaneWorkerPool();

    LaneWorkerPool(const LaneWorkerPool&) = delete;
    LaneWorkerPool& operator=(const LaneWorkerPool&) = delete;

    /** @brief Runs one task for every lane and waits for deterministic completion. */
    void run(const std::function<void(std::size_t)>& task);

    [[nodiscard]] std::size_t size() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace fil::sim
