#pragma once

#include <iostream>
#include <string_view>

namespace fil::test {

inline int failures = 0;

/** @brief Records and reports a failed test assertion. */
inline void check(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

} // namespace fil::test
