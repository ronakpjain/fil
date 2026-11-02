#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace fil {

/** Append-only scratch list for trivially copyable values. Small lists stay
 * inside their owner; overflow grows geometrically without retaining old buffers.
 */
template <typename T, std::size_t Capacity>
class InlineVector {
    static_assert(Capacity != 0U);
    static_assert(std::is_trivially_copyable_v<T>);
    static_assert(!std::is_same_v<T, bool>, "use InlineBitset for boolean sets");
public:
    InlineVector() { data_ = inline_.data(); }
    InlineVector(const InlineVector&) = delete;
    InlineVector& operator=(const InlineVector&) = delete;
    InlineVector(InlineVector&&) = delete;
    InlineVector& operator=(InlineVector&&) = delete;
    void push_back(T value) {
        if (size_ == capacity_) {
            constexpr auto maximum = std::numeric_limits<std::size_t>::max() / sizeof(T);
            if (capacity_ > maximum / 2U) throw std::length_error("inline scratch list capacity exhausted");
            const auto next_capacity = capacity_ * 2U;
            auto next = std::make_unique_for_overwrite<T[]>(next_capacity);
            std::copy_n(data_, size_, next.get());
            overflow_ = std::move(next); data_ = overflow_.get(); capacity_ = next_capacity;
        }
        data_[size_++] = value;
    }
    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] bool empty() const noexcept { return size_ == 0U; }
    [[nodiscard]] T* data() noexcept { return data_; }
    [[nodiscard]] const T* data() const noexcept { return data_; }
    T& operator[](std::size_t i) noexcept { return data_[i]; }
    const T& operator[](std::size_t i) const noexcept { return data_[i]; }
    T* begin() noexcept { return data_; }
    T* end() noexcept { return data_ + size_; }
    const T* begin() const noexcept { return data_; }
    const T* end() const noexcept { return data_ + size_; }
private:
    std::size_t size_{0U}, capacity_{Capacity};
    T* data_{nullptr};
    std::unique_ptr<T[]> overflow_;
    std::array<T, Capacity> inline_;
};

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
        overflow_(size > Capacity ? std::make_unique<T[]>(size) : nullptr) {
        data_ = overflow_ ? overflow_.get() : inline_.data();
    }
    InlineBuffer(const InlineBuffer&) = delete;
    InlineBuffer& operator=(const InlineBuffer&) = delete;
    InlineBuffer(InlineBuffer&&) = delete;
    InlineBuffer& operator=(InlineBuffer&&) = delete;
    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] T* data() noexcept { return data_; }
    [[nodiscard]] const T* data() const noexcept { return data_; }
    T& operator[](std::size_t i) noexcept { return data()[i]; }
    const T& operator[](std::size_t i) const noexcept { return data()[i]; }
    T* begin() noexcept { return data(); }
    T* end() noexcept { return data() + size_; }
    const T* begin() const noexcept { return data(); }
    const T* end() const noexcept { return data() + size_; }
private:
    std::size_t size_;
    T* data_{nullptr};
    std::unique_ptr<T[]> overflow_;
    std::array<T, Capacity> inline_{};
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
