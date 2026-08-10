#include "fil/mem/mmio_router.hpp"

#include "fil/common/numeric.hpp"

#include <algorithm>
#include <utility>

namespace fil::mem {
namespace {

Error routeError(std::string message) {
    return Error{ErrorCategory::invalid_argument, std::move(message), std::nullopt};
}

} // namespace

MmioRouter::MmioRouter(
    const std::uint32_t absolute_base,
    const std::uint32_t window_size,
    std::string name,
    const bool lenient,
    const std::uint64_t unknown_read_value
) : absolute_base_(absolute_base),
    window_size_(window_size),
    name_(std::move(name)),
    lenient_(lenient),
    unknown_read_value_(unknown_read_value) {}

Result<void> MmioRouter::map(
    const std::uint32_t offset,
    const std::uint32_t size,
    MmioDevice& device,
    std::string name
) {
    if (size == 0U || !rangeFits(offset, size, window_size_)) {
        return routeError("MMIO route '" + name + "' has an empty or out-of-window range");
    }
    const std::uint64_t end = static_cast<std::uint64_t>(offset) + size;
    for (const Route& route : routes_) {
        const std::uint64_t route_end = static_cast<std::uint64_t>(route.offset) + route.size;
        if (std::max<std::uint64_t>(offset, route.offset) < std::min(end, route_end)) {
            return routeError("MMIO route '" + name + "' overlaps route '" + route.name + "'");
        }
    }
    routes_.push_back(Route{offset, size, &device, std::move(name)});
    std::sort(routes_.begin(), routes_.end(), [](const Route& left, const Route& right) {
        return left.offset < right.offset;
    });
    return {};
}

const MmioRouter::Route* MmioRouter::find(
    const std::uint32_t offset,
    const std::uint32_t width
) const {
    const std::uint64_t end = static_cast<std::uint64_t>(offset) + width;
    for (const Route& route : routes_) {
        if (offset >= route.offset && end <= static_cast<std::uint64_t>(route.offset) + route.size) {
            return &route;
        }
        if (route.offset > offset) {
            break;
        }
    }
    return nullptr;
}

MmioDomain MmioRouter::domain(
    const std::uint32_t offset, const AccessSize size
) const noexcept {
    const std::uint32_t width = byteCount(size);
    if (!rangeFits(offset, width, window_size_)) return MmioDomain::board_local;
    const Route* const route = find(offset, width);
    if (route == nullptr) return MmioDomain::board_local;
    return route->device->domain(offset - route->offset, size);
}

UnknownMmioAccess& MmioRouter::record(const std::uint32_t absolute_address) {
    const auto found = std::lower_bound(
        unknown_.begin(), unknown_.end(), absolute_address,
        [](const UnknownMmioAccess& access, const std::uint32_t address) {
            return access.address < address;
        }
    );
    if (found != unknown_.end() && found->address == absolute_address) {
        return *found;
    }
    return *unknown_.insert(found, UnknownMmioAccess{absolute_address, 0, 0});
}

BusFault MmioRouter::fault(
    const std::uint32_t offset,
    const AccessSize size,
    const AccessContext& context,
    std::string message
) const {
    return BusFault{
        BusFaultReason::device_error,
        absolute_base_ + offset,
        size,
        context,
        name_,
        std::move(message),
    };
}

MemoryResult<std::uint64_t> MmioRouter::read(
    const std::uint32_t offset,
    const AccessSize size,
    const AccessContext& context
) {
    const std::uint32_t width = byteCount(size);
    if (!rangeFits(offset, width, window_size_)) {
        return fault(offset, size, context, "MMIO read is outside the router window");
    }
    if (const Route* route = find(offset, width)) {
        auto result = route->device->read(offset - route->offset, size, context);
        if (!result && result.fault().region.empty()) {
            result.fault().region = route->name;
        }
        return result;
    }
    if (!lenient_) {
        return fault(offset, size, context, "read from unknown MMIO register");
    }
    ++record(absolute_base_ + offset).reads;
    return unknown_read_value_;
}

MemoryResult<std::uint64_t> MmioRouter::write(
    const std::uint32_t offset,
    const AccessSize size,
    const std::uint64_t value,
    const AccessContext& context
) {
    const std::uint32_t width = byteCount(size);
    if (!rangeFits(offset, width, window_size_)) {
        return fault(offset, size, context, "MMIO write is outside the router window");
    }
    if (const Route* route = find(offset, width)) {
        auto result = route->device->write(offset - route->offset, size, value, context);
        if (!result && result.fault().region.empty()) {
            result.fault().region = route->name;
        }
        return result;
    }
    if (!lenient_) {
        return fault(offset, size, context, "write to unknown MMIO register");
    }
    ++record(absolute_base_ + offset).writes;
    return std::uint64_t{0};
}

std::vector<UnknownMmioAccess> MmioRouter::unknownAccesses() const {
    return unknown_;
}

} // namespace fil::mem
