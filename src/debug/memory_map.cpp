#include "memory_map.hpp"

#include "fil/common/format.hpp"
#include "fil/mem/memory_bus.hpp"
#include "fil/sim/board.hpp"
#include "rsp.hpp"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace fil::debug::detail {
namespace {

std::optional<std::string> memoryMapXml(const sim::Board& board) {
    std::vector<mem::MemoryRegionInfo> regions = board.memory().regions();
    std::sort(regions.begin(), regions.end(), [](const auto& left, const auto& right) {
        return left.base < right.base;
    });
    std::string xml =
        "<?xml version=\"1.0\"?>\n"
        "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" "
        "\"http://sourceware.org/gdb/gdb-memory-map.dtd\">\n"
        "<memory-map>\n";
    for (const mem::MemoryRegionInfo& region : regions) {
        std::string_view type;
        switch (region.kind) {
        case mem::RegionKind::ram: type = "ram"; break;
        case mem::RegionKind::rom: type = "rom"; break;
        case mem::RegionKind::flash: type = "flash"; break;
        case mem::RegionKind::alias:
        case mem::RegionKind::mmio: continue;
        }
        if (region.size == 0U || !region.readable) continue;
        xml += "  <memory type=\"";
        xml += type;
        xml += "\" start=\"";
        xml += fil::hexValue(region.base);
        xml += "\" length=\"";
        xml += fil::hexValue(region.size);
        if (region.kind == mem::RegionKind::flash) {
            // STM32G4 flash erases in 2 KiB pages; GDB requires a block size
            // as a child property, not an attribute.
            xml += "\">\n    <property name=\"blocksize\">0x800</property>\n  </memory>\n";
        } else {
            xml += "\"/>\n";
        }
    }
    xml += "</memory-map>\n";
    return xml;
}

} // namespace

std::optional<std::string> readMemoryMapPacket(
    const sim::Board& board,
    const std::string_view packet
) {
    constexpr std::string_view prefix = "qXfer:memory-map:read:";
    if (!packet.starts_with(prefix)) return std::nullopt;
    const std::string_view spec = packet.substr(prefix.size());
    // The memory map has an empty annex: `qXfer:memory-map:read::OFFSET,LENGTH`.
    if (spec.empty() || spec.front() != ':') return std::string("E01");
    const std::string_view range = spec.substr(1U);
    const auto comma = range.find(',');
    if (comma == std::string_view::npos) return std::string("E01");
    const auto offset = parseHex(range.substr(0U, comma));
    const auto length = parseHex(range.substr(comma + 1U));
    if (!offset || !length) return std::string("E01");
    const std::string xml = *memoryMapXml(board);
    if (*offset >= xml.size()) return std::string("l");
    const std::size_t available = xml.size() - static_cast<std::size_t>(*offset);
    const std::size_t count = std::min<std::size_t>(
        static_cast<std::size_t>(*length), available
    );
    std::string reply;
    reply.reserve(count + 1U);
    reply.push_back(count < available ? 'm' : 'l');
    reply.append(xml.substr(static_cast<std::size_t>(*offset), count));
    return reply;
}

} // namespace fil::debug::detail
