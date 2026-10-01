#include "fil/cli/network_protocol.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <vector>

namespace {

using fil::cli::network_protocol::Frame;
using fil::cli::network_protocol::FrameDecoder;
using fil::cli::network_protocol::MessageKind;

std::vector<std::uint8_t> encodedFrame(const Frame& frame) {
    std::ostringstream output(std::ios::out | std::ios::binary);
    EXPECT_TRUE(fil::cli::network_protocol::writeFrame(output, frame));
    const std::string bytes = output.str();
    return {bytes.begin(), bytes.end()};
}

TEST(NetworkProtocolTest, EncodesFixedHeaderInLittleEndian) {
    const Frame frame{MessageKind::can_inject, 0x12345678U, {0xaaU, 0x00U}};
    const std::vector<std::uint8_t> expected{
        'F', 'I', 'L', 'N', 1U, 1U, 0U, 0U,
        2U, 0U, 0U, 0U, 0x78U, 0x56U, 0x34U, 0x12U,
        0xaaU, 0x00U,
    };
    EXPECT_EQ(encodedFrame(frame), expected);
}

TEST(NetworkProtocolTest, DecodesFramesAcrossEveryHeaderAndPayloadSplit) {
    const Frame expected{MessageKind::trace, 17U, {0x00U, 0x7fU, 0xffU, 0x22U}};
    const std::vector<std::uint8_t> encoded = encodedFrame(expected);
    for (std::size_t split = 1U; split < encoded.size(); ++split) {
        FrameDecoder decoder;
        ASSERT_TRUE(decoder.append(std::span<const std::uint8_t>{encoded.data(), split}));
        auto first = decoder.next();
        ASSERT_TRUE(first) << (first ? "" : first.error().message);
        EXPECT_FALSE(first.value().has_value());
        ASSERT_TRUE(decoder.append(std::span<const std::uint8_t>{
            encoded.data() + split, encoded.size() - split
        }));
        auto decoded = decoder.next();
        ASSERT_TRUE(decoded) << (decoded ? "" : decoded.error().message);
        ASSERT_TRUE(decoded.value().has_value());
        EXPECT_EQ(decoded.value()->kind, expected.kind);
        EXPECT_EQ(decoded.value()->request_id, expected.request_id);
        EXPECT_EQ(decoded.value()->payload, expected.payload);
        EXPECT_TRUE(decoder.finish());
    }
}

TEST(NetworkProtocolTest, DecodesCoalescedFramesInOrder) {
    const Frame first{MessageKind::stop, 1U, {}};
    const Frame second{MessageKind::adc_set, 2U, {1U, 2U, 3U}};
    auto bytes = encodedFrame(first);
    const auto second_bytes = encodedFrame(second);
    bytes.insert(bytes.end(), second_bytes.begin(), second_bytes.end());

    FrameDecoder decoder;
    ASSERT_TRUE(decoder.append(bytes));
    auto first_result = decoder.next();
    ASSERT_TRUE(first_result && first_result.value().has_value());
    EXPECT_EQ(first_result.value()->kind, first.kind);
    auto second_result = decoder.next();
    ASSERT_TRUE(second_result && second_result.value().has_value());
    EXPECT_EQ(second_result.value()->kind, second.kind);
    EXPECT_EQ(second_result.value()->payload, second.payload);
    auto no_more = decoder.next();
    ASSERT_TRUE(no_more);
    EXPECT_FALSE(no_more.value().has_value());
    EXPECT_TRUE(decoder.finish());
}

TEST(NetworkProtocolTest, RejectsInvalidHeadersAndOversizedPayloads) {
    const Frame valid{MessageKind::stop, 1U, {}};
    const auto encoded = encodedFrame(valid);
    for (const std::size_t index : {0U, 4U, 6U, 8U}) {
        auto malformed = encoded;
        if (index == 0U) malformed[0U] = 'X';
        else if (index == 4U) malformed[4U] = 2U;
        else if (index == 6U) malformed[6U] = 1U;
        else malformed[8U] = 1U; // A non-empty length with no payload is incomplete, not invalid.
        FrameDecoder decoder;
        ASSERT_TRUE(decoder.append(malformed));
        auto result = decoder.next();
        if (index == 8U) {
            ASSERT_TRUE(result);
            EXPECT_FALSE(result.value().has_value());
        } else {
            EXPECT_FALSE(result);
        }
    }

    auto oversized = encoded;
    oversized[8U] = 1U;
    oversized[9U] = 0U;
    oversized[10U] = 1U;
    oversized[11U] = 0U;
    FrameDecoder decoder;
    ASSERT_TRUE(decoder.append(oversized));
    EXPECT_FALSE(decoder.next());
}

TEST(NetworkProtocolTest, DistinguishesCleanEofFromTruncatedFrame) {
    FrameDecoder empty;
    EXPECT_TRUE(empty.finish());

    const auto bytes = encodedFrame(Frame{MessageKind::can_inject, 1U, {1U, 2U}});
    FrameDecoder partial;
    ASSERT_TRUE(partial.append(std::span<const std::uint8_t>{bytes.data(), bytes.size() - 1U}));
    auto pending = partial.next();
    ASSERT_TRUE(pending);
    EXPECT_FALSE(pending.value().has_value());
    EXPECT_FALSE(partial.finish());
}

TEST(NetworkProtocolTest, AcceptsTheMaximumPayloadSize) {
    const Frame expected{MessageKind::trace, 9U,
        std::vector<std::uint8_t>(fil::cli::network_protocol::maximum_payload_size, 0x5aU)};
    const auto encoded = encodedFrame(expected);
    ASSERT_EQ(encoded.size(), fil::cli::network_protocol::header_size
                                  + fil::cli::network_protocol::maximum_payload_size);

    FrameDecoder decoder;
    ASSERT_TRUE(decoder.append(encoded));
    auto decoded = decoder.next();
    ASSERT_TRUE(decoded && decoded.value().has_value());
    EXPECT_EQ(decoded.value()->payload.size(), expected.payload.size());
    EXPECT_EQ(decoded.value()->payload.front(), 0x5aU);
    EXPECT_EQ(decoded.value()->payload.back(), 0x5aU);
    EXPECT_TRUE(decoder.finish());
}

TEST(NetworkProtocolTest, EnforcesPayloadAndBufferBounds) {
    Frame too_large{MessageKind::trace, 1U,
        std::vector<std::uint8_t>(fil::cli::network_protocol::maximum_payload_size + 1U)};
    std::ostringstream output(std::ios::out | std::ios::binary);
    EXPECT_FALSE(fil::cli::network_protocol::writeFrame(output, too_large));

    FrameDecoder decoder;
    const std::vector<std::uint8_t> oversized_input(
        fil::cli::network_protocol::maximum_buffered_size + 1U, 0U);
    EXPECT_FALSE(decoder.append(oversized_input));
}

} // namespace
