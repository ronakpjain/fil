#include "fil/common/inline_buffer.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace {
struct Guard {
    explicit Guard(unsigned& live) : live_(live) { ++live_; }
    ~Guard() { --live_; }
    Guard(const Guard&) = delete;
    Guard& operator=(const Guard&) = delete;
    unsigned& live_;
};

TEST(InlineBufferTest, NonmovableGuardsUnwindInInlineAndOverflowStorage) {
    for (const auto size : {0U,1U,8U,9U,65U}) {
        unsigned live = 0U;
        try {
            fil::InlineBuffer<std::optional<Guard>, 8U> buffer(size);
            EXPECT_EQ(buffer.size(), size);
            const auto object = reinterpret_cast<std::uintptr_t>(&buffer);
            const auto elements = reinterpret_cast<std::uintptr_t>(buffer.data());
            EXPECT_EQ(elements >= object && elements < object + sizeof(buffer), size <= 8U);
            for (auto& slot : buffer) slot.emplace(live);
            EXPECT_EQ(live, size);
            throw std::runtime_error("callback");
        } catch (const std::runtime_error&) {}
        EXPECT_EQ(live, 0U);
    }
}

TEST(InlineBufferTest, PackedFlagsCoverEmptyPartialAndOverflowWords) {
    for (const auto size : {0U,1U,63U,64U,65U,127U,128U,129U})
    for (const bool initial : {false,true}) {
        fil::InlineBitset<64U> bits(size, initial);
        EXPECT_EQ(bits.any(), initial && size != 0U);
        for (unsigned i = 0U; i < size; ++i) EXPECT_EQ(bits[i], initial);
        for (unsigned i = 0U; i < size; ++i) bits.set(i, i % 3U == 0U);
        for (unsigned i = 0U; i < size; ++i) EXPECT_EQ(bits[i], i % 3U == 0U);
        EXPECT_EQ(bits.any(), size != 0U);
        for (unsigned i = 0U; i < size; ++i) bits.set(i, false);
        EXPECT_FALSE(bits.any());
    }
}
} // namespace
