#pragma once

/** @file result.hpp
 *  @brief Value-or-error return type used throughout fil.
 */

#include "fil/common/error.hpp"

#include <cassert>
#include <utility>
#include <variant>

namespace fil {

/**
 * @brief Holds either a successful value or a structured Error.
 * @tparam T Successful value type.
 *
 * Accessing the inactive alternative is a programming error guarded by an
 * assertion. Call hasValue() or use the explicit Boolean conversion first.
 */
template <typename T>
class [[nodiscard]] Result {
public:
    /** @brief Constructs a successful result. @param value Result value. */
    Result(T value) : storage_(std::move(value)) {}

    /** @brief Constructs a failed result. @param error Failure details. */
    Result(Error error) : storage_(std::move(error)) {}

    /** @brief Tests whether this result contains a value. @return True on success. */
    [[nodiscard]] bool hasValue() const noexcept {
        return std::holds_alternative<T>(storage_);
    }

    /** @brief Tests whether this result succeeded. @return True on success. */
    [[nodiscard]] explicit operator bool() const noexcept { return hasValue(); }

    /** @brief Gets the successful value. @return Mutable lvalue reference to the value. */
    [[nodiscard]] T& value() & {
        assert(hasValue());
        return std::get<T>(storage_);
    }

    /** @brief Gets the successful value. @return Const lvalue reference to the value. */
    [[nodiscard]] const T& value() const& {
        assert(hasValue());
        return std::get<T>(storage_);
    }

    /** @brief Moves the value from an rvalue result. @return Rvalue reference to the value. */
    [[nodiscard]] T&& value() && {
        assert(hasValue());
        return std::get<T>(std::move(storage_));
    }

    /** @brief Gets the failure details. @return Mutable reference to the error. */
    [[nodiscard]] Error& error() & {
        assert(!hasValue());
        return std::get<Error>(storage_);
    }

    /** @brief Gets the failure details. @return Const reference to the error. */
    [[nodiscard]] const Error& error() const& {
        assert(!hasValue());
        return std::get<Error>(storage_);
    }

private:
    std::variant<T, Error> storage_;
};

/**
 * @brief Success-or-error specialization for operations with no return value.
 */
template <>
class [[nodiscard]] Result<void> {
public:
    /** @brief Constructs a successful result with no value. */
    Result() = default;

    /** @brief Constructs a failed result. @param error Failure details. */
    Result(Error error) : error_(std::move(error)) {}

    /** @brief Tests whether the operation succeeded. @return True on success. */
    [[nodiscard]] bool hasValue() const noexcept { return !error_.has_value(); }

    /** @brief Tests whether the operation succeeded. @return True on success. */
    [[nodiscard]] explicit operator bool() const noexcept { return hasValue(); }

    /** @brief Gets the failure details. @return Const reference to the error. */
    [[nodiscard]] const Error& error() const& {
        assert(error_.has_value());
        return *error_;
    }

private:
    std::optional<Error> error_;
};

} // namespace fil
