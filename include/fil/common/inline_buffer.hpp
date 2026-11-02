#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>

namespace fil {

/** Runtime-sized scratch storage with a stack-resident inline capacity.
 * Size and addresses stay fixed for the lifetime, including for nonmovable
 * RAII elements. Larger inputs allocate one contiguous overflow buffer.
 */
template <typename T, std::size_t Capacity>
class InlineBuffer {
    static_assert(Capacity != 0U);
    static_assert(!std::is_same_v<T, bool>, "use InlineBitset for boolean sets");
public:
    explicit InlineBuffer(std::size_t size) : size_(size),
        overflow_(size > Capacity ? std::make_unique<T[]>(size) : nullptr) {}
    InlineBuffer(const InlineBuffer&) = delete;
    InlineBuffer& operator=(const InlineBuffer&) = delete;
    InlineBuffer(InlineBuffer&&) = delete;
    InlineBuffer& operator=(InlineBuffer&&) = delete;
    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] T* data() noexcept { return overflow_ ? overflow_.get() : inline_.data(); }
    [[nodiscard]] const T* data() const noexcept { return overflow_ ? overflow_.get() : inline_.data(); }
    T& operator[](std::size_t i) noexcept { return data()[i]; }
    const T& operator[](std::size_t i) const noexcept { return data()[i]; }
    T* begin() noexcept { return data(); }
    T* end() noexcept { return data() + size_; }
    const T* begin() const noexcept { return data(); }
    const T* end() const noexcept { return data() + size_; }
private:
    std::size_t size_;
    std::array<T, Capacity> inline_{};
    std::unique_ptr<T[]> overflow_;
};

/** Packed scratch flags. Mutations must be synchronized between threads;
 * distinct bits in the same word are not independent memory locations.
 */
template <std::size_t Capacity>
class InlineBitset {
public:
    explicit InlineBitset(std::size_t size, bool value = false)
        : words_(size / 64U + (size % 64U != 0U)) {
        if (value) {
            std::fill(words_.begin(), words_.end(), UINT64_MAX);
            if (size % 64U != 0U) words_[words_.size() - 1U] &= (std::uint64_t{1U} << (size % 64U)) - 1U;
        }
    }
    [[nodiscard]] bool operator[](std::size_t bit) const noexcept {
        return (words_[bit / 64U] & (std::uint64_t{1U} << (bit % 64U))) != 0U;
    }
    void set(std::size_t bit, bool value = true) noexcept {
        const auto mask = std::uint64_t{1U} << (bit % 64U);
        if (value) words_[bit / 64U] |= mask;
        else words_[bit / 64U] &= ~mask;
    }
    [[nodiscard]] bool any() const noexcept {
        return std::any_of(words_.begin(), words_.end(), [](auto word) { return word != 0U; });
    }
private:
    InlineBuffer<std::uint64_t, (Capacity + 63U) / 64U> words_;
};

} // namespace fil
