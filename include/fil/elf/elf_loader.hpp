#pragma once

/** @file elf_loader.hpp
 *  @brief Firmware-agnostic ELF32 ARM image inspection and load metadata.
 */

#include "fil/common/result.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace fil::elf {

/**
 * @brief One validated ELF `PT_LOAD` program header.
 *
 * Bare-metal images have two address domains: physical_address stores initial
 * bytes in the load image (often flash), while virtual_address identifies where
 * those bytes live at runtime (for example `.data` in SRAM).
 */
struct ElfLoadSegment {
    std::uint32_t virtual_address{0};  ///< Runtime virtual address (`p_vaddr`).
    std::uint32_t physical_address{0}; ///< Load-memory address (`p_paddr`).
    std::uint32_t memory_size{0};      ///< Runtime allocation size (`p_memsz`).
    std::uint32_t file_size{0};        ///< File-backed byte count (`p_filesz`).
    std::uint32_t file_offset{0};      ///< Byte offset of segment data in the ELF.
    bool readable{false};              ///< ELF readable permission flag.
    bool writable{false};              ///< ELF writable permission flag.
    bool executable{false};            ///< ELF executable permission flag.
};

/** @brief Named ELF symbol retained for diagnostics and address symbolization. */
struct ElfSymbol {
    std::uint32_t address{0}; ///< Target symbol value/address.
    std::uint32_t size{0};    ///< Symbol extent in bytes when provided.
    std::string name;         ///< Null-terminated name resolved from the string table.
    bool function{false};     ///< True when ELF identifies the symbol as `STT_FUNC`.
};

/**
 * @brief Recognized values from the optional `.ARM.attributes` section.
 *
 * Missing attributes remain empty; callers must not infer that an absent tag is
 * equivalent to a particular architecture capability.
 */
struct ArmAttributes {
    std::optional<std::string> cpu_name;              ///< `Tag_CPU_name` text.
    std::optional<std::uint32_t> cpu_arch;             ///< Encoded `Tag_CPU_arch` value.
    std::optional<std::uint32_t> thumb_isa_use;        ///< Encoded `Tag_THUMB_ISA_use` value.
    std::optional<std::uint32_t> fp_arch;              ///< Encoded `Tag_FP_arch` value.
    std::optional<std::uint32_t> hard_fp_use;          ///< Encoded `Tag_ABI_HardFP_use` value.
    std::optional<std::uint32_t> vfp_args;             ///< Encoded `Tag_ABI_VFP_args` value.
    std::optional<std::uint32_t> unaligned_access;     ///< Encoded `Tag_CPU_unaligned_access` value.
};

/**
 * @brief Parsed immutable ELF metadata and original file bytes.
 *
 * ElfImage is produced by load(). Keeping the source bytes allows the memory
 * layer to materialize physical load images without reparsing or relying on
 * section headers, so stripped firmware remains supported.
 */
class ElfImage {
public:
    /** @brief Gets the ELF header entry point. @return Entry-point address. */
    [[nodiscard]] std::uint32_t entryPoint() const noexcept { return entry_point_; }

    /** @brief Gets the raw ARM ELF flags. @return ELF `e_flags` value. */
    [[nodiscard]] std::uint32_t flags() const noexcept { return flags_; }

    /** @brief Gets the selected vector-table load address. @return Vector-table base. */
    [[nodiscard]] std::uint32_t vectorBase() const noexcept { return vector_base_; }

    /** @brief Gets the initial main stack pointer from the vector table. @return Initial MSP. */
    [[nodiscard]] std::uint32_t initialMsp() const noexcept { return initial_msp_; }

    /** @brief Gets the reset-handler vector including its Thumb bit. @return Reset vector. */
    [[nodiscard]] std::uint32_t resetHandler() const noexcept { return reset_handler_; }

    /** @brief Tests the ELF hard-float ABI flag. @return True for the hard-float ABI. */
    [[nodiscard]] bool hardFloatAbi() const noexcept { return (flags_ & 0x400U) != 0; }

    /** @brief Gets parsed PT_LOAD records. @return Segments sorted by load address. */
    [[nodiscard]] const std::vector<ElfLoadSegment>& segments() const noexcept { return segments_; }

    /** @brief Gets parsed named symbols. @return Symbols sorted by address. */
    [[nodiscard]] const std::vector<ElfSymbol>& symbols() const noexcept { return symbols_; }

    /** @brief Gets recognized `.ARM.attributes` values. @return ARM attributes. */
    [[nodiscard]] const ArmAttributes& armAttributes() const noexcept { return attributes_; }

    /** @brief Gets the immutable original ELF bytes. @return View into image-owned storage. */
    [[nodiscard]] std::span<const std::uint8_t> fileBytes() const noexcept { return file_bytes_; }

    /**
     * @brief Reads bytes by physical/load address, never runtime address.
     * @param address First physical address to read.
     * @param size Number of bytes to read.
     * @return Copied bytes, or an error when the range is not file-backed.
     */
    [[nodiscard]] Result<std::vector<std::uint8_t>> readLoadImage(
        std::uint32_t address,
        std::uint32_t size
    ) const;
    /**
     * @brief Finds the nearest symbol whose address does not exceed a target.
     * @param address Address to symbolize.
     * @return Pointer into image-owned symbol storage, or `nullptr` if none precedes it.
     */
    [[nodiscard]] const ElfSymbol* symbolAtOrBefore(std::uint32_t address) const noexcept;

private:
    friend Result<ElfImage> load(const std::filesystem::path&, std::optional<std::uint32_t>);

    std::filesystem::path path_;
    std::vector<std::uint8_t> file_bytes_;
    std::vector<ElfLoadSegment> segments_;
    std::vector<ElfSymbol> symbols_;
    ArmAttributes attributes_;
    std::uint32_t entry_point_{0};
    std::uint32_t flags_{0};
    std::uint32_t vector_base_{0};
    std::uint32_t initial_msp_{0};
    std::uint32_t reset_handler_{0};
};

/**
 * @brief Loads and validates an ELF32 little-endian ARM executable.
 * @param path Firmware ELF path.
 * @param vector_base Optional physical vector-table address override.
 * @return Parsed immutable image, or a source-aware parse/IO error.
 */
[[nodiscard]] Result<ElfImage> load(
    const std::filesystem::path& path,
    std::optional<std::uint32_t> vector_base = std::nullopt
);

/**
 * @brief Formats parsed ELF metadata for command-line inspection.
 * @param image Parsed ELF image.
 * @param path Display path associated with the image.
 * @return Stable multiline inspection report.
 */
[[nodiscard]] std::string inspect(const ElfImage& image, const std::filesystem::path& path);

} // namespace fil::elf
