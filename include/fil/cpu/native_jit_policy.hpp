#pragma once

#include <cstdint>
#include <limits>

namespace fil::cpu {

/** Deterministic allocation-free policy; caller supplies matched timing windows. */
class NativeJitPolicy {
public:
    enum class State : std::uint8_t { cached, native, cooldown, rejected };
    struct Config {
        std::uint8_t confidence_windows{4U};
        std::uint8_t cooldown_windows{8U};
        std::uint16_t min_improvement_percent{12U};
        std::uint64_t min_window_ns{1000U};
        // Observed retired work supplies a bounded forecast, not paid-down debt.
        std::uint16_t reuse_projection{16U};
    };

    constexpr NativeJitPolicy() noexcept = default;
    constexpr explicit NativeJitPolicy(Config config) noexcept : config_(config) {}
    [[nodiscard]] constexpr State state() const noexcept { return state_; }
    [[nodiscard]] constexpr std::uint64_t kernelCompileCostNs() const noexcept {
        return kernel_compile_cost_ns_;
    }
    [[nodiscard]] constexpr std::uint32_t reuseForecast() const noexcept { return reuse_forecast_; }
    [[nodiscard]] constexpr bool bootstrapComplete() const noexcept {
        return observed_windows_ >= 8U;
    }
    constexpr void addKernelCompileCost(std::uint64_t ns) noexcept {
        kernel_compile_cost_ns_ = saturatingAdd(kernel_compile_cost_ns_, ns);
    }
    constexpr void noteReuse(std::uint32_t count = 1U) noexcept {
        const auto projected = static_cast<std::uint64_t>(count) * config_.reuse_projection;
        const auto remaining = max_reuse_forecast - reuse_forecast_;
        reuse_forecast_ += static_cast<std::uint32_t>(projected < remaining ? projected : remaining);
    }
    [[nodiscard]] constexpr bool hasTimingWindow(std::uint64_t cached_ns,
        std::uint64_t native_ns, std::uint32_t instructions) const noexcept {
        // Do not discard a strong win just because the faster side is short.
        return instructions != 0U && cached_ns != 0U && native_ns != 0U
            && (cached_ns >= config_.min_window_ns || native_ns >= config_.min_window_ns);
    }
    constexpr void reject() noexcept {
        state_ = State::rejected;
        winners_ = losers_ = cooldown_ = 0U;
    }

    /** Matched totals for equal shadow instruction counts; shadow work is not reuse. */
    constexpr State observe(std::uint64_t cached_ns, std::uint64_t native_ns,
                            std::uint32_t instructions) noexcept {
        if (state_ == State::rejected || !hasTimingWindow(cached_ns, native_ns, instructions))
            return state_;
        if (observed_windows_ < 8U) ++observed_windows_;
        if (cooldown_ != 0U) {
            --cooldown_;
            return state_;
        }
        if (executionProfitable(cached_ns, native_ns)) {
            losers_ = 0U;
            if (winners_ < config_.confidence_windows) ++winners_;
            // A fast but not-yet-amortizable kernel is not a timing loser.
            // Retain its confidence while actual reuse grows the forecast.
            if (state_ != State::native && winners_ >= config_.confidence_windows
                && compileCostRecoverable(cached_ns - native_ns, instructions)) {
                state_ = State::native;
            }
        } else {
            winners_ = 0U;
            if (losers_ < config_.confidence_windows) ++losers_;
            if (state_ == State::native && losers_ >= config_.confidence_windows) {
                state_ = State::cooldown;
                cooldown_ = config_.cooldown_windows;
                losers_ = 0U;
            } else if (state_ == State::cached && losers_ >= config_.confidence_windows) {
                losers_ = 0U;
                cooldown_ = config_.cooldown_windows;
            }
        }
        return state_;
    }

private:
    static constexpr std::uint32_t max_reuse_forecast = 16'777'216U;
    [[nodiscard]] constexpr bool executionProfitable(std::uint64_t cached_ns,
        std::uint64_t native_ns) const noexcept {
        if (config_.min_improvement_percent > 100U || cached_ns <= native_ns) return false;
        const auto required = (cached_ns / 100U) * config_.min_improvement_percent
            + ((cached_ns % 100U) * config_.min_improvement_percent) / 100U;
        return cached_ns - native_ns >= required;
    }

    /** Exact fraction comparison using Euclidean division; no overflowing products. */
    [[nodiscard]] static constexpr bool fractionLessEqual(std::uint64_t a, std::uint64_t b,
        std::uint64_t c, std::uint64_t d) noexcept {
        bool reversed = false;
        for (;;) {
            const auto left = a / b;
            const auto right = c / d;
            if (left != right) return reversed ? left > right : left < right;
            a %= b;
            c %= d;
            if (a == 0U || c == 0U) {
                if (a == 0U && c == 0U) return true;
                return reversed ? c == 0U : a == 0U;
            }
            const auto old_b = b;
            const auto old_d = d;
            b = a;
            d = c;
            a = old_b;
            c = old_d;
            reversed = !reversed;
        }
    }
    [[nodiscard]] constexpr bool compileCostRecoverable(std::uint64_t window_saving_ns,
        std::uint32_t instructions) const noexcept {
        if (kernel_compile_cost_ns_ == 0U) return true;
        if (reuse_forecast_ == 0U || window_saving_ns == 0U || instructions == 0U) return false;
        // cost * sampled instructions <= window savings * future instructions.
        // Preserve sub-nanosecond/instruction savings instead of truncating them
        // to zero; compare fractions to keep UINT64_MAX inputs well defined.
        return fractionLessEqual(kernel_compile_cost_ns_, window_saving_ns,
                                 reuse_forecast_, instructions);
    }
    [[nodiscard]] static constexpr std::uint64_t saturatingAdd(
        std::uint64_t a, std::uint64_t b) noexcept {
        return b > std::numeric_limits<std::uint64_t>::max() - a
            ? std::numeric_limits<std::uint64_t>::max() : a + b;
    }
    Config config_{};
    State state_{State::cached};
    std::uint8_t winners_{0U};
    std::uint8_t losers_{0U};
    std::uint8_t cooldown_{0U};
    std::uint8_t observed_windows_{0U};
    std::uint32_t reuse_forecast_{0U};
    std::uint64_t kernel_compile_cost_ns_{0U};
};

} // namespace fil::cpu
