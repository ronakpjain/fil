#include "fil/cortexm/system_control.hpp"
#include "fil/stm32g4/peripheral.hpp"
#include "fil/stm32g4/stm32g4.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <span>
#include <vector>

namespace {

constexpr fil::mem::AccessContext read_context{fil::mem::AccessType::data_read, 0};
constexpr fil::mem::AccessContext write_context{fil::mem::AccessType::data_write, 0};

constexpr std::uint32_t crc_dr = 0x00;
constexpr std::uint32_t crc_cr = 0x08;
constexpr std::uint32_t crc_init = 0x10;
constexpr std::uint32_t crc_pol = 0x14;

/** @brief Independent byte-wise CRC-32/MPEG-2 reference (no reflection, no final XOR). */
std::uint32_t referenceCrc32Mpeg2(std::span<const std::uint8_t> bytes) {
    std::uint32_t crc = 0xffffffffU;
    for (const std::uint8_t byte : bytes) {
        crc ^= static_cast<std::uint32_t>(byte) << 24U;
        for (unsigned int bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x80000000U) != 0U
                ? (crc << 1U) ^ 0x04c11db7U
                : (crc << 1U);
        }
    }
    return crc;
}

/** @brief Independent byte-wise CRC reference with a caller-chosen polynomial. */
std::uint32_t referenceCrc32Mpeg2(
    std::span<const std::uint8_t> bytes,
    const std::uint32_t polynomial,
    const std::uint32_t initial
) {
    std::uint32_t crc = initial;
    for (const std::uint8_t byte : bytes) {
        crc ^= static_cast<std::uint32_t>(byte) << 24U;
        for (unsigned int bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x80000000U) != 0U
                ? (crc << 1U) ^ polynomial
                : (crc << 1U);
        }
    }
    return crc;
}

/** @brief Verifies the emulated CRC matches an independent CRC-32/MPEG-2 reference. */
TEST(CrcPeripheralTest, MatchesCrc32Mpeg2Reference) {
    fil::stm32g4::CrcPeripheral crc;

    // Anchor the reference implementation to the standard check value.
    const std::vector<std::uint8_t> check{
        '1', '2', '3', '4', '5', '6', '7', '8', '9',
    };
    EXPECT_EQ(referenceCrc32Mpeg2(check), 0x0376e6e7U)
        << "reference implements CRC-32/MPEG-2";

    // Feed "123456789" as 32-bit words; the final word is zero-padded.
    EXPECT_TRUE(crc.write(crc_cr, fil::mem::AccessSize::word, 1U, write_context).hasValue())
        << "CR RESET reloads the default initial value";
    EXPECT_TRUE(
        crc.write(crc_dr, fil::mem::AccessSize::word, 0x31323334U, write_context).hasValue());
    EXPECT_TRUE(
        crc.write(crc_dr, fil::mem::AccessSize::word, 0x35363738U, write_context).hasValue());
    EXPECT_TRUE(
        crc.write(crc_dr, fil::mem::AccessSize::word, 0x39000000U, write_context).hasValue());

    const std::vector<std::uint8_t> padded{
        '1', '2', '3', '4', '5', '6', '7', '8', '9', 0, 0, 0,
    };
    const auto result = crc.read(crc_dr, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(result.hasValue()) << "CRC DR read succeeds";
    EXPECT_EQ(result.value(), referenceCrc32Mpeg2(padded))
        << "word-wise DR writes match the byte-wise reference";
    EXPECT_EQ(crc.crc(), result.value()) << "accessor exposes the current CRC";
}

/** @brief Verifies CR RESET and CR INIT reload the configured initial value. */
TEST(CrcPeripheralTest, ResetsAndReloadsInit) {
    fil::stm32g4::CrcPeripheral crc;
    auto initial = crc.read(crc_dr, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(initial && initial.value() == 0xffffffffU)
        << "CRC starts at the default 0xFFFFFFFF initial value";

    EXPECT_TRUE(
        crc.write(crc_dr, fil::mem::AccessSize::word, 0x12345678U, write_context).hasValue());
    const auto advanced = crc.read(crc_dr, fil::mem::AccessSize::word, read_context);
    ASSERT_TRUE(advanced.hasValue());
    EXPECT_NE(advanced.value(), 0xffffffffU) << "feeding DR advances the CRC";

    EXPECT_TRUE(crc.write(crc_cr, fil::mem::AccessSize::word, 1U, write_context).hasValue())
        << "sets the CR RESET bit";
    const auto reset = crc.read(crc_dr, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(reset && reset.value() == 0xffffffffU)
        << "CR RESET restores the initial value";

    EXPECT_TRUE(crc.write(crc_init, fil::mem::AccessSize::word, 0xaaaaaaaaU, write_context)
            .hasValue())
        << "stores a new INIT register value";
    EXPECT_TRUE(crc.write(crc_cr, fil::mem::AccessSize::word, 1U << 7U, write_context).hasValue())
        << "sets the CR INIT bit";
    const auto reloaded = crc.read(crc_dr, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(reloaded && reloaded.value() == 0xaaaaaaaaU)
        << "CR INIT reloads the INIT register into DR";
    EXPECT_EQ(crc.initValue(), 0xaaaaaaaaU) << "exposes the configured initial value";

    ASSERT_TRUE(crc.write(crc_pol, fil::mem::AccessSize::word, 0x1021U, write_context).hasValue());
    crc.reset();
    EXPECT_EQ(crc.crc(), 0xffffffffU);
    EXPECT_EQ(crc.initValue(), 0xffffffffU);
    EXPECT_EQ(crc.polynomial(), 0x04c11db7U);
}

/** @brief Verifies the POL register changes the computed polynomial. */
TEST(CrcPeripheralTest, HonorsPolynomialRegister) {
    fil::stm32g4::CrcPeripheral crc;
    constexpr std::uint32_t custom_poly = 0x1021U; // CRC-16/CCITT-style width kept as 32-bit
    EXPECT_TRUE(crc.write(crc_pol, fil::mem::AccessSize::word, custom_poly, write_context)
            .hasValue())
        << "stores a custom POL register value";
    EXPECT_TRUE(crc.write(crc_cr, fil::mem::AccessSize::word, 1U, write_context).hasValue())
        << "resets before the custom computation";
    EXPECT_TRUE(
        crc.write(crc_dr, fil::mem::AccessSize::word, 0x31323334U, write_context).hasValue());

    const std::vector<std::uint8_t> bytes{'1', '2', '3', '4'};
    const auto result = crc.read(crc_dr, fil::mem::AccessSize::word, read_context);
    ASSERT_TRUE(result.hasValue());
    EXPECT_EQ(result.value(), referenceCrc32Mpeg2(bytes, custom_poly, 0xffffffffU))
        << "POL register drives the CRC polynomial";
    EXPECT_EQ(crc.polynomial(), custom_poly) << "exposes the configured polynomial";
}

/** @brief Verifies the integrated STM32G4 routes CRC register accesses. */
TEST(CrcIntegrationTest, RoutesThroughStm32G4) {
    fil::sim::EventLoop events;
    fil::sim::TraceRecorder trace;
    fil::cortexm::SystemControl system;
    auto mcu = fil::stm32g4::Stm32G4::create(events, trace, system, true, 16'000'000U);
    ASSERT_TRUE(mcu.hasValue());

    const auto initial =
        mcu.value()->router().read(0x00023000U, fil::mem::AccessSize::word, read_context);
    EXPECT_TRUE(initial && initial.value() == 0xffffffffU);
    EXPECT_TRUE(
        mcu.value()->router()
            .write(0x00023000U, fil::mem::AccessSize::word, 0x31323334U, write_context)
            .hasValue());
}


} // namespace
