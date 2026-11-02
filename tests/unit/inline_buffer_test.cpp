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

TEST(InlineBufferTest, AppendOnlyListsKeepInlineStorageAndGrowWithoutLosingFixups) {
    struct Fixup { std::uint32_t position{0U}, label{0U}; };
    fil::InlineVector<Fixup, 4U> list;
    const auto object = reinterpret_cast<std::uintptr_t>(&list);
    const auto elements = reinterpret_cast<std::uintptr_t>(list.data());
    EXPECT_TRUE(list.empty());
    ASSERT_GE(elements, object); ASSERT_LT(elements, object + sizeof(list));
    const auto* inline_data = list.data();
    for (std::uint32_t i = 0U; i < 65U; ++i) {
        list.push_back({i * 7U, i * 11U});
        EXPECT_EQ(list.size(), i + 1U);
        EXPECT_EQ(list.data() == inline_data, i < 4U);
        for (std::uint32_t j = 0U; j <= i; ++j) {
            EXPECT_EQ(list[j].position, j * 7U); EXPECT_EQ(list[j].label, j * 11U);
        }
    }
    const auto& view = list;
    std::uint32_t i = 0U;
    for (const auto& fixup : view) EXPECT_EQ(fixup.label, i++ * 11U);
    EXPECT_EQ(view.data(), list.data());
    // A source element must remain valid when appending triggers growth.
    fil::InlineVector<std::uint8_t, 1U> bytes;
    bytes.push_back(0xa5U); bytes.push_back(bytes[0]);
    EXPECT_EQ(bytes[0], 0xa5U); EXPECT_EQ(bytes[1], 0xa5U);
}

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
