#include "fil/sim/event_loop.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fil::sim {
struct EventLoop::Impl {
    /**
     * @brief Callback storage for one scheduled event.
     *
     * Slots are reused after cancellation or execution. The monotonically
     * increasing EventId is the slot's generation: stale heap entries compare
     * their stored id against the slot and are discarded without an ABA risk.
     */
    struct Slot {
        SimTimeNs at{0};
        EventId id{0};
        std::uint64_t sequence{0};
        EventOwner owner{shared_event_owner};
        EventCallback callback;
        BuiltinEvent builtin{};
        std::function<bool()> locality_guard;
        bool globally_observed{true};
        bool live{false};
    };

    /** Key and slot reference carried by every pending queue entry. */
    struct Entry {
        SimTimeNs at{0};
        std::uint64_t sequence{0};
        std::uint32_t slot{0};
        EventId id{0};
    };

    struct SlotRef {
        std::uint32_t slot{0};
        EventId id{0};
    };

    struct Later {
        bool operator()(const Entry& left, const Entry& right) const noexcept {
            if (left.at != right.at) return left.at > right.at;
            return left.sequence > right.sequence;
        }
    };

    using Queue = std::priority_queue<Entry, std::vector<Entry>, Later>;

    SimTimeNs shared_now{0};
    EventId next_id{1};
    std::uint64_t next_sequence{0};
    std::size_t maximum_same_time_events{100000};
    mutable std::recursive_mutex mutex;
    EventOwner serial_active_owner{shared_event_owner};
    std::vector<Slot> slots;
    std::vector<std::uint32_t> free_slots;
    Queue events;
    Queue globally_observed_events;
    static constexpr std::size_t dense_owner_count = 64U;
    std::array<Queue, dense_owner_count> dense_owner_events;
    std::unordered_map<EventOwner, Queue> owner_events;
    std::shared_ptr<const ObservationBarrier> observation_barrier;
    ObservationBarrier scheduler_observation_barrier;
    std::unordered_map<EventId, SlotRef> live_events;
    std::array<std::optional<SimTimeNs>, dense_owner_count> dense_owner_now;
    std::unordered_map<EventOwner, SimTimeNs> owner_now;
    std::uint64_t executed_callbacks{0U};
    bool owner_dispatch_active{false};
    std::size_t global_dispatch_depth{0U};
    bool lane_domain{false};
    EventOwner lane_owner{shared_event_owner};
    std::uint64_t lane_next_id{1U};
    std::vector<std::unique_ptr<Impl>> lane_domains;
    std::size_t lane_owner_count{0U};
    bool lane_domains_active{false};
    std::atomic<bool> lane_domains_orphaned{false};
    std::thread::id lane_coordinator{};
    std::uint64_t lane_counter_baseline{0U};
    std::array<std::uint64_t, dense_owner_count> next_lane_ids = [] {
        std::array<std::uint64_t, dense_owner_count> ids{};
        ids.fill(1U);
        return ids;
    }();
    std::atomic<std::size_t> active_lane_scopes{0U};
    std::array<std::atomic<bool>, dense_owner_count> lane_owner_active{};
    std::recursive_mutex coordinator_gate_mutex;

    static constexpr std::uint64_t lane_id_marker = std::uint64_t{1U} << 63U;
    static constexpr std::uint64_t lane_id_counter_mask = (std::uint64_t{1U} << 56U) - 1U;
    static constexpr std::uint64_t sequence_partition = std::uint64_t{1U} << 56U;

    static Impl*& exclusiveImpl() noexcept {
        static thread_local Impl* current = nullptr;
        return current;
    }

    struct Lock {
        explicit Lock(Impl& impl) : lock(impl.mutex, std::defer_lock) {
            if (exclusiveImpl() != &impl) lock.lock();
            if (!impl.lane_domain && impl.lane_domains_active) {
                coordinator_gate = std::unique_lock<std::recursive_mutex>(impl.coordinator_gate_mutex);
                if (impl.active_lane_scopes.load(std::memory_order_acquire) != 0U) {
                    throw std::logic_error("root EventLoop operation requires parked lane workers");
                }
            }
        }
        std::unique_lock<std::recursive_mutex> lock;
        std::unique_lock<std::recursive_mutex> coordinator_gate;
    };

    struct OwnerDispatchGuard {
        explicit OwnerDispatchGuard(Impl& implementation) : impl(implementation) {
            if (impl.owner_dispatch_active) {
                throw std::logic_error("cannot reenter event dispatch during an owner run");
            }
            impl.owner_dispatch_active = true;
        }
        ~OwnerDispatchGuard() { impl.owner_dispatch_active = false; }

        OwnerDispatchGuard(const OwnerDispatchGuard&) = delete;
        OwnerDispatchGuard& operator=(const OwnerDispatchGuard&) = delete;

        Impl& impl;
    };

    struct GlobalDispatchGuard {
        explicit GlobalDispatchGuard(Impl& implementation) : impl(implementation) {
            ++impl.global_dispatch_depth;
        }
        ~GlobalDispatchGuard() { --impl.global_dispatch_depth; }
        GlobalDispatchGuard(const GlobalDispatchGuard&) = delete;
        GlobalDispatchGuard& operator=(const GlobalDispatchGuard&) = delete;
        Impl& impl;
    };

    struct CoordinatorGateGuard {
        explicit CoordinatorGateGuard(Impl& root) {
            if (root.lane_domains_active && !root.lane_domain) {
                lock = std::unique_lock<std::recursive_mutex>(root.coordinator_gate_mutex);
                if (root.active_lane_scopes.load(std::memory_order_acquire) != 0U) {
                    throw std::logic_error("coordinator access requires parked lane workers");
                }
            }
        }
        std::unique_lock<std::recursive_mutex> lock;
    };

    struct OwnerContextGuard {
        OwnerContextGuard(Impl& implementation, const EventOwner owner)
            : impl(implementation), previous(impl.currentOwner()) {
            impl.setCurrentOwner(owner);
        }
        ~OwnerContextGuard() { impl.setCurrentOwner(previous); }
        OwnerContextGuard(const OwnerContextGuard&) = delete;
        OwnerContextGuard& operator=(const OwnerContextGuard&) = delete;
        Impl& impl;
        EventOwner previous;
    };

    [[nodiscard]] EventOwner currentOwner() const noexcept {
        return serial_active_owner;
    }

    void setCurrentOwner(const EventOwner owner) noexcept {
        serial_active_owner = owner;
    }

    [[nodiscard]] SimTimeNs timeFor(const EventOwner owner) const noexcept {
        if (owner == shared_event_owner) return shared_now;
        if (owner < dense_owner_count) {
            const auto& local_time = dense_owner_now[owner];
            if (!local_time) return shared_now;
            return std::max(*local_time, shared_now);
        }
        const auto found = owner_now.find(owner);
        if (found == owner_now.end()) return shared_now;
        // Owner clocks never read behind the shared clock: runDueEvents used
        // to force every owner forward on each call, which cost a map walk
        // per scheduler frontier. Clamping here returns the same value that
        // eager walk would have produced (no owner read can intervene between
        // a rewind and the next frontier advance without passing through a
        // runDueEvents that re-syncs first), while frontiers with no owner
        // activity skip the walk entirely.
        return std::max(found->second, shared_now);
    }

    void setTime(const EventOwner owner, const SimTimeNs time) {
        if (owner == shared_event_owner) {
            shared_now = time;
        } else if (owner < dense_owner_count) {
            dense_owner_now[owner] = time;
        } else {
            owner_now[owner] = time;
        }
    }

    [[nodiscard]] bool entryLive(const Entry& entry) const noexcept {
        return entry.slot < slots.size()
            && slots[entry.slot].live
            && slots[entry.slot].id == entry.id;
    }

    static void discardDeadFront(Queue& queue, const std::vector<Slot>& slots) {
        while (!queue.empty()) {
            const Entry& entry = queue.top();
            if (entry.slot < slots.size() && slots[entry.slot].live
                && slots[entry.slot].id == entry.id) return;
            queue.pop();
        }
    }

    void discardDeadFront(Queue& queue) const {
        discardDeadFront(queue, slots);
    }

    void discardDeadGlobalFront() {
        discardDeadFront(events);
    }

    [[nodiscard]] Queue* ownerQueue(const EventOwner owner) {
        if (owner < dense_owner_count) {
            Queue& queue = dense_owner_events[owner];
            discardDeadFront(queue);
            return &queue;
        }
        const auto found = owner_events.find(owner);
        if (found == owner_events.end()) return nullptr;
        discardDeadFront(found->second);
        return &found->second;
    }

    [[nodiscard]] Queue& schedulingQueue(const EventOwner owner) {
        if (owner < dense_owner_count) return dense_owner_events[owner];
        return owner_events[owner];
    }

    [[nodiscard]] EventId allocateId() {
        if (lane_domain) {
            if (lane_next_id == 0U || lane_next_id > lane_id_counter_mask) {
                throw std::overflow_error("lane event identifier space exhausted");
            }
            const EventId id = lane_id_marker
                | (static_cast<EventId>(lane_owner) << 56U) | lane_next_id;
            ++lane_next_id;
            return id;
        }
        if (next_id == 0U || next_id >= lane_id_marker) {
            throw std::overflow_error("simulation event identifier space exhausted");
        }
        return next_id++;
    }

    [[nodiscard]] std::uint64_t allocateSequence() {
        if (next_sequence == std::numeric_limits<std::uint64_t>::max()) {
            throw std::overflow_error("simulation event sequence space exhausted");
        }
        return next_sequence++;
    }

    void rebuildQueues() {
        events = {};
        globally_observed_events = {};
        for (auto& queue : dense_owner_events) queue = {};
        owner_events.clear();
        for (const auto& [id, ref] : live_events) {
            if (ref.slot >= slots.size()) continue;
            const Slot& slot = slots[ref.slot];
            if (!slot.live || slot.id != id) continue;
            const Entry entry{slot.at, slot.sequence, ref.slot, id};
            events.push(entry);
            if (slot.globally_observed) globally_observed_events.push(entry);
            schedulingQueue(slot.owner).push(entry);
        }
    }

    void swapEventStorage(Impl& other) noexcept {
        slots.swap(other.slots);
        free_slots.swap(other.free_slots);
        events.swap(other.events);
        globally_observed_events.swap(other.globally_observed_events);
        dense_owner_events.swap(other.dense_owner_events);
        owner_events.swap(other.owner_events);
        live_events.swap(other.live_events);
    }

    [[nodiscard]] bool cancelId(const EventId id) noexcept {
        if (id == 0U) return false;
        const auto found = live_events.find(id);
        if (found == live_events.end()) return false;
        const SlotRef ref = found->second;
        if (ref.slot >= slots.size() || slots[ref.slot].id != ref.id) {
            live_events.erase(found);
            return false;
        }
        releaseSlot(ref.slot);
        live_events.erase(found);
        discardDeadFront(globally_observed_events);
        return true;
    }

    void releaseSlot(const std::uint32_t index) noexcept {
        Slot& slot = slots[index];
        slot.live = false;
        slot.callback = {};
        slot.builtin = {};
        slot.locality_guard = {};
        free_slots.push_back(index);
    }

    void markOwner(EventRunResult& result, const EventOwner owner) const noexcept {
        if (owner < 64U) result.local_owner_mask |= std::uint64_t{1U} << owner;
        else result.shared_event_executed = true;
    }

    std::uint32_t acquireSlot() {
        std::uint32_t index = 0;
        if (!free_slots.empty()) {
            index = free_slots.back();
            free_slots.pop_back();
        } else {
            if (slots.size() >= std::numeric_limits<std::uint32_t>::max()) {
                throw std::length_error("simulation event slot space exhausted");
            }
            index = static_cast<std::uint32_t>(slots.size());
            slots.emplace_back();
        }
        slots[index].live = true;
        return index;
    }

    EventId schedule(
        const SimTimeNs at, EventCallback callback,
        std::function<bool()> locality_guard = {}
    ) {
        if (!callback) throw std::invalid_argument("simulation event callback is empty");
        const EventOwner owner = currentOwner();
        if (lane_domain && owner != lane_owner) {
            throw std::logic_error("lane domain cannot schedule another owner's event");
        }
        if (at < timeFor(owner)) {
            throw std::invalid_argument("cannot schedule a simulation event in the past");
        }
        const EventId id = allocateId();
        const bool globally_observed = !locality_guard || owner == shared_event_owner;
        const std::uint32_t index = acquireSlot();
        Slot& slot = slots[index];
        slot.at = at;
        slot.id = id;
        slot.sequence = allocateSequence();
        slot.owner = owner;
        slot.callback = std::move(callback);
        slot.builtin = {};
        slot.locality_guard = std::move(locality_guard);
        slot.globally_observed = globally_observed;

        const Entry entry{slot.at, slot.sequence, index, id};
        events.push(entry);
        if (globally_observed) globally_observed_events.push(entry);
        Queue& owner_queue = schedulingQueue(owner);
        discardDeadFront(owner_queue);
        owner_queue.push(entry);
        live_events.emplace(id, SlotRef{index, id});
        return id;
    }

    EventId scheduleBuiltin(
        const SimTimeNs at, const BuiltinEvent event,
        std::function<bool()> locality_guard = {}
    ) {
        if (!event) throw std::invalid_argument("simulation built-in callback is empty");
        const EventOwner owner = currentOwner();
        if (lane_domain && owner != lane_owner) {
            throw std::logic_error("lane domain cannot schedule another owner's event");
        }
        if (at < timeFor(owner)) {
            throw std::invalid_argument("cannot schedule a simulation event in the past");
        }
        const EventId id = allocateId();
        const bool globally_observed = !locality_guard || owner == shared_event_owner;
        const std::uint32_t index = acquireSlot();
        Slot& slot = slots[index];
        slot.at = at;
        slot.id = id;
        slot.sequence = allocateSequence();
        slot.owner = owner;
        slot.callback = {};
        slot.builtin = event;
        slot.locality_guard = std::move(locality_guard);
        slot.globally_observed = globally_observed;

        const Entry entry{slot.at, slot.sequence, index, id};
        events.push(entry);
        if (globally_observed) globally_observed_events.push(entry);
        Queue& owner_queue = schedulingQueue(owner);
        discardDeadFront(owner_queue);
        owner_queue.push(entry);
        live_events.emplace(id, SlotRef{index, id});
        return id;
    }

    EventRunResult runDueEventsLocked(const SimTimeNs deadline) {
        if (deadline < shared_now) {
            throw std::invalid_argument("cannot run the simulation clock backwards");
        }
        EventRunResult result{0, false, shared_now};
        SimTimeNs counted_time = 0U;
        std::size_t events_at_counted_time = 0U;
        for (;;) {
            discardDeadGlobalFront();
            if (events.empty() || events.top().at > deadline) {
                shared_now = deadline;
                result.stopped_at = deadline;
                return result;
            }
            const Entry entry = events.top();
            const SimTimeNs next_time = entry.at;
            if (events_at_counted_time == 0U || next_time != counted_time) {
                counted_time = next_time;
                events_at_counted_time = 0U;
            }
            if (events_at_counted_time >= maximum_same_time_events) {
                shared_now = next_time;
                result.same_time_limit_hit = true;
                result.stopped_at = next_time;
                return result;
            }
            events.pop();
            shared_now = entry.at;
            ++events_at_counted_time;
            invoke(entry, result);
        }
    }

    void invoke(const Entry& entry, EventRunResult& result) {
        Slot& slot = slots[entry.slot];
        const SimTimeNs at = slot.at;
        const EventOwner owner = slot.owner;
        const EventId id = slot.id;
        EventCallback callback = std::move(slot.callback);
        const BuiltinEvent builtin = slot.builtin;
        std::function<bool()> locality_guard = std::move(slot.locality_guard);
        slot.live = false;
        live_events.erase(id);
        free_slots.push_back(entry.slot);
        discardDeadFront(globally_observed_events);

        setTime(owner, at);
        markOwner(result, owner);
        ++result.events_executed;
        const EventOwner previous_owner = currentOwner();
        setCurrentOwner(owner);
        try {
            const std::shared_ptr<const ObservationBarrier> observer = observation_barrier;
            const bool owner_local = owner != shared_event_owner
                && locality_guard && locality_guard();
            const EventObservation classification = owner_local
                ? EventObservation::owner_local : EventObservation::global;
            if (scheduler_observation_barrier) {
                scheduler_observation_barrier(
                    at, owner, observer ? EventObservation::global : classification
                );
            }
            if (observer && *observer) (*observer)(at, owner, classification);
            if (owner_local && !locality_guard()) {
                if (scheduler_observation_barrier) {
                    scheduler_observation_barrier(at, owner, EventObservation::global);
                }
                if (observer && *observer) (*observer)(at, owner, EventObservation::global);
            }
            if (builtin) builtin.invoke(builtin.context);
            else callback();
            if (executed_callbacks != std::numeric_limits<std::uint64_t>::max()) {
                ++executed_callbacks;
            }
        } catch (...) {
            setCurrentOwner(previous_owner);
            throw;
        }
        setCurrentOwner(previous_owner);
    }

    void beginLaneDomains(const std::size_t owner_count) {
        if (lane_domains_active || lane_domain) {
            throw std::logic_error("lane domains are already active");
        }
        if (owner_count == 0U || owner_count > dense_owner_count) {
            throw std::invalid_argument("lane domain owner count must be in [1, 64]");
        }
        if (owner_dispatch_active || global_dispatch_depth != 0U) {
            throw std::logic_error("cannot migrate events during dispatch");
        }
        if (next_id >= lane_id_marker || next_sequence >= sequence_partition) {
            throw std::overflow_error("event identifier or sequence space cannot be partitioned");
        }

        struct Transfer {
            const Slot* source;
            Slot* destination;
        };
        std::vector<std::unique_ptr<Impl>> domains;
        domains.reserve(owner_count);
        std::vector<std::size_t> counts(owner_count, 0U);
        for (const auto& [id, ref] : live_events) {
            static_cast<void>(id);
            if (ref.slot < slots.size() && slots[ref.slot].live
                && slots[ref.slot].owner < owner_count) {
                ++counts[slots[ref.slot].owner];
            }
        }

        Impl prepared_root;
        prepared_root.shared_now = shared_now;
        prepared_root.next_id = next_id;
        prepared_root.next_sequence = next_sequence;
        prepared_root.maximum_same_time_events = maximum_same_time_events;
        prepared_root.observation_barrier = observation_barrier;
        prepared_root.scheduler_observation_barrier = scheduler_observation_barrier;
        std::vector<Transfer> transfers;
        transfers.reserve(live_events.size());
        for (std::size_t owner = 0; owner < owner_count; ++owner) {
            auto domain = std::make_unique<Impl>();
            domain->lane_domain = true;
            domain->lane_owner = static_cast<EventOwner>(owner);
            domain->lane_next_id = next_lane_ids[owner];
            domain->shared_now = shared_now;
            domain->dense_owner_now[owner] = dense_owner_now[owner];
            domain->maximum_same_time_events = maximum_same_time_events;
            domain->observation_barrier = observation_barrier;
            domain->scheduler_observation_barrier = scheduler_observation_barrier;
            domain->next_sequence = next_sequence
                + (static_cast<std::uint64_t>(owner) + 1U) * sequence_partition;
            domain->executed_callbacks = executed_callbacks;
            domain->slots.reserve(counts[owner]);
            domain->live_events.reserve(counts[owner]);
            domains.push_back(std::move(domain));
        }
        prepared_root.slots.reserve(live_events.size());
        prepared_root.live_events.reserve(live_events.size());

        // Prepare fully allocated destination metadata and queues by copying
        // callable objects. Original mutable callable state is transferred only
        // after every map, vector, and heap allocation has succeeded.
        for (const auto& [id, ref] : live_events) {
            if (ref.slot >= slots.size()) continue;
            const Slot& source = slots[ref.slot];
            if (!source.live || source.id != id) continue;
            Impl& destination = source.owner < owner_count
                ? *domains[source.owner] : prepared_root;
            const auto destination_index = static_cast<std::uint32_t>(destination.slots.size());
            destination.slots.push_back(source);
            destination.live_events.emplace(id, SlotRef{destination_index, id});
            transfers.push_back(Transfer{&source, &destination.slots.back()});
        }
        prepared_root.rebuildQueues();
        for (auto& domain : domains) domain->rebuildQueues();

        // No-throw commit: move the original callback/guard state to its prepared
        // destination, then swap storage and publish the immutable domain table.
        for (const Transfer& transfer : transfers) {
            transfer.destination->callback = std::move(const_cast<Slot*>(transfer.source)->callback);
            transfer.destination->locality_guard =
                std::move(const_cast<Slot*>(transfer.source)->locality_guard);
        }
        swapEventStorage(prepared_root);
        lane_domains = std::move(domains);
        lane_counter_baseline = executed_callbacks;
        lane_owner_count = owner_count;
        lane_coordinator = std::this_thread::get_id();
        lane_domains_orphaned.store(false, std::memory_order_release);
        lane_domains_active = true;
    }

    void finishLaneDomains() {
        if (!lane_domains_active) return;
        if (active_lane_scopes.load(std::memory_order_acquire) != 0U) {
            throw std::logic_error("cannot close lane domains while workers are active");
        }

        struct Source {
            Impl* domain;
            std::uint32_t source_index;
            std::uint32_t destination_index;
            EventId id;
        };
        Impl prepared_root;
        prepared_root.shared_now = shared_now;
        prepared_root.next_id = next_id;
        prepared_root.maximum_same_time_events = maximum_same_time_events;
        prepared_root.observation_barrier = observation_barrier;
        prepared_root.scheduler_observation_barrier = scheduler_observation_barrier;

        std::vector<Source> sources;
        std::size_t total_live = live_events.size();
        for (const auto& domain : lane_domains) total_live += domain->live_events.size();
        sources.reserve(total_live);
        prepared_root.slots.reserve(total_live);
        prepared_root.live_events.reserve(total_live);
        auto append = [&](Impl& source, const EventId id, const SlotRef ref) {
            if (ref.slot >= source.slots.size()) return;
            const Slot& slot = source.slots[ref.slot];
            if (!slot.live || slot.id != id) return;
            const auto destination_index = static_cast<std::uint32_t>(prepared_root.slots.size());
            prepared_root.slots.push_back(slot);
            prepared_root.live_events.emplace(id, SlotRef{destination_index, id});
            sources.push_back(Source{&source, ref.slot, destination_index, id});
        };
        for (const auto& [id, ref] : live_events) append(*this, id, ref);
        for (auto& domain : lane_domains) {
            for (const auto& [id, ref] : domain->live_events) append(*domain, id, ref);
        }

        std::sort(sources.begin(), sources.end(), [](const Source& left, const Source& right) {
            const auto& left_slot = left.domain->slots[left.source_index];
            const auto& right_slot = right.domain->slots[right.source_index];
            return left_slot.sequence < right_slot.sequence;
        });
        for (std::size_t index = 0; index < sources.size(); ++index) {
            prepared_root.slots[sources[index].destination_index].sequence =
                static_cast<std::uint64_t>(index);
        }
        prepared_root.rebuildQueues();

        std::uint64_t total_callbacks = lane_counter_baseline;
        const auto add_count = [&](const std::uint64_t count) {
            total_callbacks = count > std::numeric_limits<std::uint64_t>::max() - total_callbacks
                ? std::numeric_limits<std::uint64_t>::max() : total_callbacks + count;
        };
        add_count(executed_callbacks >= lane_counter_baseline
            ? executed_callbacks - lane_counter_baseline : 0U);
        for (const auto& domain : lane_domains) {
            add_count(domain->executed_callbacks >= lane_counter_baseline
                ? domain->executed_callbacks - lane_counter_baseline : 0U);
        }
        std::array<std::uint64_t, dense_owner_count> merged_lane_ids = next_lane_ids;
        std::array<std::optional<SimTimeNs>, dense_owner_count> merged_clocks = dense_owner_now;
        std::size_t merged_maximum = maximum_same_time_events;
        ObservationBarrier merged_observer = observation_barrier
            ? *observation_barrier : ObservationBarrier{};
        auto merged_observer_ptr = merged_observer
            ? std::make_shared<const ObservationBarrier>(merged_observer) : nullptr;
        ObservationBarrier merged_scheduler = scheduler_observation_barrier;
        for (const auto& domain : lane_domains) {
            merged_lane_ids[domain->lane_owner] = domain->lane_next_id;
            merged_clocks[domain->lane_owner] = domain->dense_owner_now[domain->lane_owner];
            if (domain->lane_owner == 0U) {
                merged_maximum = domain->maximum_same_time_events;
                merged_observer = domain->observation_barrier
                    ? *domain->observation_barrier : ObservationBarrier{};
                merged_scheduler = domain->scheduler_observation_barrier;
            }
        }
        // All allocations/copies are complete. The following callback moves and
        // storage swaps are non-allocating; failed preparation leaves the live
        // root and all domains untouched for retry.
        for (const Source& source : sources) {
            Slot& original = source.domain->slots[source.source_index];
            Slot& destination = prepared_root.slots[source.destination_index];
            destination.callback = std::move(original.callback);
            destination.locality_guard = std::move(original.locality_guard);
        }
        swapEventStorage(prepared_root);
        dense_owner_now = merged_clocks;
        next_lane_ids = merged_lane_ids;
        next_sequence = static_cast<std::uint64_t>(sources.size());
        executed_callbacks = total_callbacks;
        maximum_same_time_events = merged_maximum;
        observation_barrier.swap(merged_observer_ptr);
        scheduler_observation_barrier.swap(merged_scheduler);
        lane_domains.clear();
        lane_owner_count = 0U;
        lane_domains_active = false;
        lane_domains_orphaned.store(false, std::memory_order_release);
        lane_coordinator = {};
    }

};

EventLoop::EventLoop(const std::size_t maximum_same_time_events)
    : impl_(std::make_unique<Impl>()) {
    impl_->maximum_same_time_events = maximum_same_time_events;
}

EventLoop::~EventLoop() {
    if (impl_ && impl_->lane_domains_active
        && !impl_->lane_domains_orphaned.load(std::memory_order_acquire)) {
        std::terminate();
    }
}

EventLoop::EventLoop(EventLoop&& other) noexcept {
    if (other.impl_ && other.impl_->lane_domains_active) std::terminate();
    impl_ = std::move(other.impl_);
}

EventLoop& EventLoop::operator=(EventLoop&& other) noexcept {
    if ((impl_ && impl_->lane_domains_active)
        || (other.impl_ && other.impl_->lane_domains_active)) {
        std::terminate();
    }
    impl_ = std::move(other.impl_);
    return *this;
}

EventLoop*& EventLoop::laneBindingLoop() noexcept {
    static thread_local EventLoop* loop = nullptr;
    return loop;
}

void*& EventLoop::laneBindingImpl() noexcept {
    static thread_local void* impl = nullptr;
    return impl;
}

EventLoop::Impl* EventLoop::resolvedImpl() const {
    if (laneBindingLoop() == this) {
        return static_cast<Impl*>(laneBindingImpl());
    }
    Impl& root = *impl_;
    if (root.lane_domains_orphaned.load(std::memory_order_acquire)) {
        std::unique_lock<std::recursive_mutex> root_lock(root.mutex);
        std::unique_lock<std::recursive_mutex> gate(root.coordinator_gate_mutex);
        root.finishLaneDomains();
    }
    Impl::Lock lock(root);
    if (root.lane_domains_active) {
        if (std::this_thread::get_id() != root.lane_coordinator
            || root.active_lane_scopes.load(std::memory_order_acquire) != 0U) {
            throw std::logic_error("root EventLoop operation requires parked lane workers");
        }
    }
    return &root;
}

void EventLoop::beginLaneDomains(const std::size_t owner_count) {
    if (impl_->lane_domains_orphaned.load(std::memory_order_acquire)) finishLaneDomains();
    Impl::Lock lock(*impl_);
    impl_->beginLaneDomains(owner_count);
}

void EventLoop::finishLaneDomains() {
    std::unique_lock<std::recursive_mutex> root_lock(impl_->mutex);
    std::unique_lock<std::recursive_mutex> gate(impl_->coordinator_gate_mutex);
    impl_->finishLaneDomains();
}

void EventLoop::orphanLaneDomains() noexcept {
    impl_->lane_domains_orphaned.store(true, std::memory_order_release);
}

EventLoop::LaneDomainsScope::LaneDomainsScope(
    EventLoop& loop, const std::size_t owner_count
) : loop_(&loop) {
    loop_->beginLaneDomains(owner_count);
}

EventLoop::LaneDomainsScope::~LaneDomainsScope() noexcept {
    if (loop_ == nullptr) return;
    try {
        close();
    } catch (...) {
        loop_->orphanLaneDomains();
    }
}

void EventLoop::LaneDomainsScope::close() {
    if (loop_ == nullptr) return;
    loop_->finishLaneDomains();
    loop_ = nullptr;
}

EventLoop::LaneDomainScope::LaneDomainScope(
    EventLoop& loop, const EventOwner owner
) : loop_(&loop), previous_loop_(EventLoop::laneBindingLoop()),
    previous_impl_(EventLoop::laneBindingImpl()), owner_(owner) {
    if (previous_loop_ != nullptr && previous_loop_ != &loop) {
        throw std::logic_error("cannot nest lane domains from different EventLoops");
    }
    Impl& root = *loop.impl_;
    // The gate excludes concurrent coordinator operations, but is independent
    // of the root mutex so a coordinator-held root exclusive lease is harmless.
    std::lock_guard<std::recursive_mutex> gate(root.coordinator_gate_mutex);
    if (!root.lane_domains_active || owner >= root.lane_owner_count) {
        throw std::logic_error("requested lane domain is not active");
    }
    if (previous_loop_ == &loop) {
        throw std::logic_error("lane domain scopes cannot be nested");
    }
    bool expected = false;
    if (!root.lane_owner_active[owner].compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
        throw std::logic_error("lane owner already has an active worker");
    }
    Impl& domain = *root.lane_domains[owner];
    domain.shared_now = std::max(domain.shared_now, root.shared_now);
    root.active_lane_scopes.fetch_add(1U, std::memory_order_release);
    EventLoop::laneBindingLoop() = &loop;
    EventLoop::laneBindingImpl() = &domain;
}

EventLoop::LaneDomainScope::~LaneDomainScope() noexcept {
    if (loop_ == nullptr) return;
    EventLoop::laneBindingLoop() = static_cast<EventLoop*>(previous_loop_);
    EventLoop::laneBindingImpl() = previous_impl_;
    Impl& root = *loop_->impl_;
    root.lane_owner_active[owner_].store(false, std::memory_order_release);
    root.active_lane_scopes.fetch_sub(1U, std::memory_order_acq_rel);
}

EventLoop::ExclusiveScope::ExclusiveScope(EventLoop& loop) {
    Impl* const selected = loop.resolvedImpl();
    mutex_ = &selected->mutex;
    impl_ = selected;
    previous_ = Impl::exclusiveImpl();
    mutex_->lock();
    Impl::exclusiveImpl() = selected;
}

EventLoop::ExclusiveScope::~ExclusiveScope() noexcept {
    if (impl_ == nullptr) return;
    Impl::exclusiveImpl() = static_cast<Impl*>(previous_);
    mutex_->unlock();
}

EventLoop::ExclusiveScope EventLoop::useExclusiveExecution() {
    return ExclusiveScope(*this);
}

EventLoop::OwnerScope::OwnerScope(EventLoop& loop, const EventOwner owner)
    : impl_(loop.resolvedImpl()), previous_(static_cast<Impl*>(impl_)->currentOwner()) {
    static_cast<Impl*>(impl_)->setCurrentOwner(owner);
}

EventLoop::OwnerScope::~OwnerScope() {
    if (impl_ != nullptr) static_cast<Impl*>(impl_)->setCurrentOwner(previous_);
}

EventLoop::OwnerScope::OwnerScope(OwnerScope&& other) noexcept
    : impl_(std::exchange(other.impl_, nullptr)), previous_(other.previous_) {}

EventLoop::OwnerScope EventLoop::useOwner(const EventOwner owner) {
    return OwnerScope(*this, owner);
}

EventLoop::LaneDomainsScope EventLoop::useLaneDomains(const std::size_t owner_count) {
    return LaneDomainsScope(*this, owner_count);
}

EventLoop::LaneDomainScope EventLoop::useLaneDomain(const EventOwner owner) {
    return LaneDomainScope(*this, owner);
}

EventOwner EventLoop::activeOwner() const {
    return resolvedImpl()->currentOwner();
}

EventId EventLoop::scheduleAfter(const SimTimeNs delta, EventCallback callback) {
    Impl* root = resolvedImpl();
    Impl::CoordinatorGateGuard coordinator_gate(*root);
    if (root->lane_domain || !root->lane_domains_active) {
        Impl::Lock lock(*root);
        const EventOwner owner = root->currentOwner();
        const SimTimeNs current = root->timeFor(owner);
        if (delta > std::numeric_limits<SimTimeNs>::max() - current) {
            throw std::overflow_error("simulation event time overflow");
        }
        return root->schedule(current + delta, std::move(callback));
    }
    const EventOwner owner = root->currentOwner();
    if (owner < root->lane_owner_count) {
        Impl& domain = *root->lane_domains[owner];
        Impl::Lock lock(domain);
        domain.shared_now = std::max(domain.shared_now, root->shared_now);
        Impl::OwnerContextGuard owner_context(domain, owner);
        const SimTimeNs current = domain.timeFor(owner);
        if (delta > std::numeric_limits<SimTimeNs>::max() - current) {
            throw std::overflow_error("simulation event time overflow");
        }
        return domain.schedule(current + delta, std::move(callback));
    }
    Impl::Lock lock(*root);
    const SimTimeNs current = root->timeFor(owner);
    if (delta > std::numeric_limits<SimTimeNs>::max() - current) {
        throw std::overflow_error("simulation event time overflow");
    }
    return root->schedule(current + delta, std::move(callback));
}

EventId EventLoop::scheduleAt(const SimTimeNs at, EventCallback callback) {
    Impl* root = resolvedImpl();
    Impl::CoordinatorGateGuard coordinator_gate(*root);
    if (root->lane_domain || !root->lane_domains_active) {
        Impl::Lock lock(*root);
        return root->schedule(at, std::move(callback));
    }
    const EventOwner owner = root->currentOwner();
    if (owner < root->lane_owner_count) {
        Impl& domain = *root->lane_domains[owner];
        Impl::Lock lock(domain);
        domain.shared_now = std::max(domain.shared_now, root->shared_now);
        Impl::OwnerContextGuard owner_context(domain, owner);
        return domain.schedule(at, std::move(callback));
    }
    Impl::Lock lock(*root);
    return root->schedule(at, std::move(callback));
}

EventId EventLoop::scheduleBuiltinAfter(const SimTimeNs delta, const BuiltinEvent event) {
    Impl* root = resolvedImpl();
    Impl::CoordinatorGateGuard coordinator_gate(*root);
    if (root->lane_domain || !root->lane_domains_active) {
        Impl::Lock lock(*root);
        const EventOwner owner = root->currentOwner();
        const SimTimeNs current = root->timeFor(owner);
        if (delta > std::numeric_limits<SimTimeNs>::max() - current) {
            throw std::overflow_error("simulation event time overflow");
        }
        return root->scheduleBuiltin(current + delta, event);
    }
    const EventOwner owner = root->currentOwner();
    if (owner < root->lane_owner_count) {
        Impl& domain = *root->lane_domains[owner];
        Impl::Lock lock(domain);
        domain.shared_now = std::max(domain.shared_now, root->shared_now);
        Impl::OwnerContextGuard owner_context(domain, owner);
        const SimTimeNs current = domain.timeFor(owner);
        if (delta > std::numeric_limits<SimTimeNs>::max() - current) {
            throw std::overflow_error("simulation event time overflow");
        }
        return domain.scheduleBuiltin(current + delta, event);
    }
    Impl::Lock lock(*root);
    const SimTimeNs current = root->timeFor(owner);
    if (delta > std::numeric_limits<SimTimeNs>::max() - current) {
        throw std::overflow_error("simulation event time overflow");
    }
    return root->scheduleBuiltin(current + delta, event);
}

EventId EventLoop::scheduleBuiltinAt(const SimTimeNs at, const BuiltinEvent event) {
    Impl* root = resolvedImpl();
    Impl::CoordinatorGateGuard coordinator_gate(*root);
    if (root->lane_domain || !root->lane_domains_active) {
        Impl::Lock lock(*root);
        return root->scheduleBuiltin(at, event);
    }
    const EventOwner owner = root->currentOwner();
    if (owner < root->lane_owner_count) {
        Impl& domain = *root->lane_domains[owner];
        Impl::Lock lock(domain);
        domain.shared_now = std::max(domain.shared_now, root->shared_now);
        Impl::OwnerContextGuard owner_context(domain, owner);
        return domain.scheduleBuiltin(at, event);
    }
    Impl::Lock lock(*root);
    return root->scheduleBuiltin(at, event);
}

EventId EventLoop::scheduleOwnerLocalAt(
    const SimTimeNs at, EventCallback callback, std::function<bool()> locality_guard
) {
    Impl* root = resolvedImpl();
    Impl::CoordinatorGateGuard coordinator_gate(*root);
    if (!locality_guard) throw std::invalid_argument("owner-local event requires a locality guard");
    if (root->lane_domain || !root->lane_domains_active) {
        Impl::Lock lock(*root);
        return root->schedule(at, std::move(callback), std::move(locality_guard));
    }
    const EventOwner owner = root->currentOwner();
    if (owner < root->lane_owner_count) {
        Impl& domain = *root->lane_domains[owner];
        Impl::Lock lock(domain);
        domain.shared_now = std::max(domain.shared_now, root->shared_now);
        Impl::OwnerContextGuard owner_context(domain, owner);
        return domain.schedule(at, std::move(callback), std::move(locality_guard));
    }
    Impl::Lock lock(*root);
    return root->schedule(at, std::move(callback), std::move(locality_guard));
}

EventId EventLoop::scheduleOwnerLocalBuiltinAt(
    const SimTimeNs at, const BuiltinEvent event, std::function<bool()> locality_guard
) {
    Impl* root = resolvedImpl();
    Impl::CoordinatorGateGuard coordinator_gate(*root);
    if (!locality_guard) throw std::invalid_argument("owner-local event requires a locality guard");
    if (root->lane_domain || !root->lane_domains_active) {
        Impl::Lock lock(*root);
        return root->scheduleBuiltin(at, event, std::move(locality_guard));
    }
    const EventOwner owner = root->currentOwner();
    if (owner < root->lane_owner_count) {
        Impl& domain = *root->lane_domains[owner];
        Impl::Lock lock(domain);
        domain.shared_now = std::max(domain.shared_now, root->shared_now);
        Impl::OwnerContextGuard owner_context(domain, owner);
        return domain.scheduleBuiltin(at, event, std::move(locality_guard));
    }
    Impl::Lock lock(*root);
    return root->scheduleBuiltin(at, event, std::move(locality_guard));
}

bool EventLoop::cancel(const EventId id) {
    Impl* state = resolvedImpl();
    if (state->lane_domain) {
        Impl::Lock lock(*state);
        return state->cancelId(id);
    }
    Impl::Lock lock(*state);
    if (state->lane_domains_active) {
        if (state->cancelId(id)) return true;
        for (auto& domain : state->lane_domains) {
            Impl::Lock domain_lock(*domain);
            if (domain->cancelId(id)) return true;
        }
        return false;
    }
    return state->cancelId(id);
}

EventRunResult EventLoop::runDueEvents(const SimTimeNs deadline) {
    Impl* state = resolvedImpl();
    Impl::Lock lock(*state);
    if (state->lane_domain || state->lane_domains_active) {
        throw std::logic_error("global event dispatch is unavailable while lane domains are active");
    }
    if (state->owner_dispatch_active) {
        throw std::logic_error("cannot run global events during an owner run");
    }
    Impl::GlobalDispatchGuard dispatch_guard(*state);
    return state->runDueEventsLocked(deadline);
}

EventRunResult EventLoop::runOwnedEvents(
    const EventOwner owner, const SimTimeNs deadline
) {
    Impl* state = resolvedImpl();
    if (state->lane_domain) {
        if (owner != state->lane_owner) {
            throw std::logic_error("lane domain cannot run another owner's events");
        }
    } else if (state->lane_domains_active && owner < state->lane_owner_count) {
        auto lane = useLaneDomain(owner);
        return runOwnedEvents(owner, deadline);
    }
    Impl::Lock lock(*state);
    Impl::OwnerDispatchGuard dispatch_guard(*state);
    struct SharedClockSync {
        Impl* state;
        EventOwner owner;
        ~SharedClockSync() {
            if (owner == shared_event_owner && !state->lane_domain
                && state->lane_domains_active) {
                for (auto& domain : state->lane_domains) domain->shared_now = state->shared_now;
            }
        }
    } sync{state, owner};
    const SimTimeNs current = state->timeFor(owner);
    if (deadline < current) {
        throw std::invalid_argument("cannot run an owner clock backwards");
    }

    EventRunResult result{0, false, current};
    SimTimeNs counted_time = 0U;
    std::size_t events_at_counted_time = 0U;
    for (;;) {
        Impl::Queue* const queue = state->ownerQueue(owner);
        if (queue == nullptr || queue->empty() || queue->top().at > deadline) {
            state->setTime(owner, deadline);
            result.stopped_at = deadline;
            return result;
        }

        const Impl::Entry entry = queue->top();
        const SimTimeNs next_time = entry.at;
        if (events_at_counted_time == 0U || next_time != counted_time) {
            counted_time = next_time;
            events_at_counted_time = 0U;
        }
        if (events_at_counted_time >= state->maximum_same_time_events) {
            state->setTime(owner, next_time);
            result.same_time_limit_hit = true;
            result.stopped_at = next_time;
            return result;
        }

        queue->pop();
        ++events_at_counted_time;
        state->invoke(entry, result);
    }
}

EventRunResult EventLoop::runSharedEvents(const SimTimeNs deadline) {
    return runOwnedEvents(shared_event_owner, deadline);
}

EventRunResult EventLoop::advanceBy(const SimTimeNs delta) {
    Impl* state = resolvedImpl();
    Impl::Lock lock(*state);
    if (state->lane_domain || state->lane_domains_active) {
        throw std::logic_error("global event advance is unavailable while lane domains are active");
    }
    if (state->owner_dispatch_active) {
        throw std::logic_error("cannot advance global events during an owner run");
    }
    if (delta > std::numeric_limits<SimTimeNs>::max() - state->shared_now) {
        throw std::overflow_error("simulation clock overflow");
    }
    Impl::GlobalDispatchGuard dispatch_guard(*state);
    return state->runDueEventsLocked(state->shared_now + delta);
}

void EventLoop::clear() {
    Impl* state = resolvedImpl();
    Impl::Lock lock(*state);
    auto clear_domain = [](Impl& domain) {
        domain.events = {};
        domain.globally_observed_events = {};
        for (auto& queue : domain.dense_owner_events) queue = {};
        domain.owner_events.clear();
        domain.live_events.clear();
        for (auto& slot : domain.slots) {
            slot.live = false;
            slot.callback = {};
            slot.builtin = {};
            slot.locality_guard = {};
        }
        domain.free_slots.resize(domain.slots.size());
        std::iota(domain.free_slots.begin(), domain.free_slots.end(), 0U);
    };
    clear_domain(*state);
    if (!state->lane_domain && state->lane_domains_active) {
        for (auto& domain : state->lane_domains) clear_domain(*domain);
    }
}

SimTimeNs EventLoop::now() const {
    Impl* state = resolvedImpl();
    Impl::CoordinatorGateGuard coordinator_gate(*state);
    if (state->lane_domain || !state->lane_domains_active) {
        Impl::Lock lock(*state);
        const EventOwner owner = state->currentOwner();
        if (state->lane_domain && owner != state->lane_owner) {
            throw std::logic_error("lane domain cannot read another owner's clock");
        }
        return state->timeFor(owner);
    }
    const EventOwner owner = state->currentOwner();
    if (owner < state->lane_owner_count) {
        Impl* const root = state;
        Impl& domain = *root->lane_domains[owner];
        Impl::Lock lock(domain);
        domain.shared_now = std::max(domain.shared_now, root->shared_now);
        return domain.timeFor(owner);
    }
    Impl::Lock lock(*state);
    return state->timeFor(owner);
}

SimTimeNs EventLoop::now(const EventOwner owner) const {
    Impl* state = resolvedImpl();
    Impl::CoordinatorGateGuard coordinator_gate(*state);
    if (state->lane_domain && owner != state->lane_owner) {
        throw std::logic_error("lane domain cannot read another owner's clock");
    }
    if (!state->lane_domain && state->lane_domains_active && owner < state->lane_owner_count) {
        Impl* const root = state;
        state = root->lane_domains[owner].get();
        Impl::Lock lock(*state);
        state->shared_now = std::max(state->shared_now, root->shared_now);
        return state->timeFor(owner);
    }
    Impl::Lock lock(*state);
    return state->timeFor(owner);
}

std::size_t EventLoop::pending() const {
    Impl* state = resolvedImpl();
    Impl::Lock lock(*state);
    std::size_t total = state->live_events.size();
    if (!state->lane_domain && state->lane_domains_active) {
        for (const auto& domain : state->lane_domains) total += domain->live_events.size();
    }
    return total;
}

std::uint64_t EventLoop::executedCallbacks() const {
    Impl* state = resolvedImpl();
    Impl::Lock lock(*state);
    if (state->lane_domain || !state->lane_domains_active) return state->executed_callbacks;
    std::uint64_t total = state->lane_counter_baseline;
    const std::uint64_t root_delta = state->executed_callbacks >= state->lane_counter_baseline
        ? state->executed_callbacks - state->lane_counter_baseline : 0U;
    total = root_delta > std::numeric_limits<std::uint64_t>::max() - total
        ? std::numeric_limits<std::uint64_t>::max() : total + root_delta;
    for (const auto& domain : state->lane_domains) {
        const auto delta = domain->executed_callbacks >= state->lane_counter_baseline
            ? domain->executed_callbacks - state->lane_counter_baseline : 0U;
        total = delta > std::numeric_limits<std::uint64_t>::max() - total
            ? std::numeric_limits<std::uint64_t>::max() : total + delta;
    }
    return total;
}

std::optional<SimTimeNs> EventLoop::nextScheduledTime() {
    Impl* state = resolvedImpl();
    if (state->lane_domain && state->currentOwner() != state->lane_owner) {
        throw std::logic_error("lane domain cannot inspect the shared event queue");
    }
    Impl::Lock lock(*state);
    auto earliest = [](Impl& domain) -> std::optional<SimTimeNs> {
        domain.discardDeadGlobalFront();
        if (domain.events.empty()) return std::nullopt;
        return domain.events.top().at;
    };
    auto result = earliest(*state);
    if (!state->lane_domain && state->lane_domains_active) {
        for (auto& domain : state->lane_domains) {
            const auto candidate = earliest(*domain);
            if (candidate && (!result || *candidate < *result)) result = candidate;
        }
    }
    return result;
}

std::optional<SimTimeNs> EventLoop::nextScheduledTime(const EventOwner owner) {
    Impl* state = resolvedImpl();
    Impl::CoordinatorGateGuard coordinator_gate(*state);
    if (state->lane_domain && owner != state->lane_owner) {
        throw std::logic_error("lane domain cannot inspect another owner's queue");
    }
    if (!state->lane_domain && state->lane_domains_active && owner < state->lane_owner_count) {
        state = state->lane_domains[owner].get();
    }
    Impl::Lock lock(*state);
    Impl::Queue* const queue = state->ownerQueue(owner);
    if (queue == nullptr || queue->empty()) return std::nullopt;
    return queue->top().at;
}

bool EventLoop::hasObservationBarrier() const {
    Impl* state = resolvedImpl();
    Impl::Lock lock(*state);
    return static_cast<bool>(state->observation_barrier)
        || static_cast<bool>(state->scheduler_observation_barrier);
}

ObservationBarrier EventLoop::exchangeObservationBarrier(ObservationBarrier barrier) {
    Impl* state = resolvedImpl();
    Impl::Lock lock(*state);
    ObservationBarrier previous;
    if (state->observation_barrier) previous = *state->observation_barrier;
    state->observation_barrier = barrier
        ? std::make_shared<const ObservationBarrier>(std::move(barrier)) : nullptr;
    if (!state->lane_domain && state->lane_domains_active) {
        for (auto& domain : state->lane_domains) domain->observation_barrier = state->observation_barrier;
    }
    return previous;
}

ObservationBarrier EventLoop::exchangeSchedulerObservationBarrier(
    ObservationBarrier barrier
) {
    Impl* state = resolvedImpl();
    Impl::Lock lock(*state);
    std::swap(state->scheduler_observation_barrier, barrier);
    if (!state->lane_domain && state->lane_domains_active) {
        for (auto& domain : state->lane_domains)
            domain->scheduler_observation_barrier = state->scheduler_observation_barrier;
    }
    return barrier;
}

std::optional<SimTimeNs> EventLoop::nextObservationTime(const EventOwner owner) {
    Impl* state = resolvedImpl();
    if (state->lane_domain && owner != state->lane_owner) {
        throw std::logic_error("lane domain cannot inspect another owner's observations");
    }
    Impl* target = state;
    if (!state->lane_domain && state->lane_domains_active && owner < state->lane_owner_count) {
        target = state->lane_domains[owner].get();
    }
    Impl::Lock lock(*state);
    Impl::Lock target_lock(*target);
    auto observe = [&](Impl& domain, std::optional<SimTimeNs>& result) {
        Impl::Queue* const owner_queue = domain.ownerQueue(owner);
        domain.discardDeadFront(domain.globally_observed_events);
        if (owner_queue != nullptr && !owner_queue->empty()) result = owner_queue->top().at;
        if (!domain.globally_observed_events.empty()) {
            const SimTimeNs candidate = domain.globally_observed_events.top().at;
            if (!result || candidate < *result) result = candidate;
        }
    };
    std::optional<SimTimeNs> result;
    observe(*target, result);
    if (!state->lane_domain && state->lane_domains_active) {
        for (auto& domain : state->lane_domains) {
            if (domain.get() != target) observe(*domain, result);
        }
        state->discardDeadFront(state->globally_observed_events);
        if (!state->globally_observed_events.empty()) {
            const SimTimeNs candidate = state->globally_observed_events.top().at;
            if (!result || candidate < *result) result = candidate;
        }
    }
    return result;
}

void EventLoop::setMaximumSameTimeEvents(const std::size_t maximum) {
    Impl* state = resolvedImpl();
    Impl::Lock lock(*state);
    state->maximum_same_time_events = maximum;
    if (!state->lane_domain && state->lane_domains_active) {
        for (auto& domain : state->lane_domains) domain->maximum_same_time_events = maximum;
    }
}

std::size_t EventLoop::maximumSameTimeEvents() const {
    Impl* state = resolvedImpl();
    Impl::Lock lock(*state);
    return state->maximum_same_time_events;
}

ScheduledEvent::~ScheduledEvent() {
    cancel();
}

EventCallback ScheduledEvent::retireBefore(EventCallback callback) {
    return [this, callback = std::move(callback)]() mutable {
        id_ = 0U;
        callback();
    };
}

EventId ScheduledEvent::scheduleAfter(
    const SimTimeNs delta,
    EventCallback callback
) {
    if (!callback) throw std::invalid_argument("scheduled event callback is empty");
    cancel();
    if (loop_ == nullptr) return 0U;
    id_ = loop_->scheduleAfter(delta, retireBefore(std::move(callback)));
    return id_;
}

EventId ScheduledEvent::scheduleAt(
    const SimTimeNs at,
    EventCallback callback
) {
    if (!callback) throw std::invalid_argument("scheduled event callback is empty");
    cancel();
    if (loop_ == nullptr) return 0U;
    id_ = loop_->scheduleAt(at, retireBefore(std::move(callback)));
    return id_;
}

void ScheduledEvent::builtinTrampoline(void* context) {
    auto* binding = static_cast<BuiltinBinding*>(context);
    binding->handle->retireInvocation();
    if (binding->event) binding->event.invoke(binding->event.context);
}

EventId ScheduledEvent::scheduleBuiltinAfter(
    const SimTimeNs delta, const BuiltinEvent event
) {
    if (!event) throw std::invalid_argument("scheduled built-in callback is empty");
    cancel();
    if (loop_ == nullptr) return 0U;
    builtin_binding_ = BuiltinBinding{this, event};
    id_ = loop_->scheduleBuiltinAfter(
        delta, {&ScheduledEvent::builtinTrampoline, &builtin_binding_});
    return id_;
}

EventId ScheduledEvent::scheduleBuiltinAt(
    const SimTimeNs at, const BuiltinEvent event
) {
    if (!event) throw std::invalid_argument("scheduled built-in callback is empty");
    cancel();
    if (loop_ == nullptr) return 0U;
    builtin_binding_ = BuiltinBinding{this, event};
    id_ = loop_->scheduleBuiltinAt(
        at, {&ScheduledEvent::builtinTrampoline, &builtin_binding_});
    return id_;
}

EventId ScheduledEvent::scheduleOwnerLocalAt(
    const SimTimeNs at, EventCallback callback, std::function<bool()> locality_guard
) {
    if (!callback) throw std::invalid_argument("scheduled event callback is empty");
    cancel();
    if (loop_ == nullptr) return 0U;
    id_ = loop_->scheduleOwnerLocalAt(
        at, retireBefore(std::move(callback)), std::move(locality_guard)
    );
    return id_;
}

EventId ScheduledEvent::scheduleOwnerLocalBuiltinAt(
    const SimTimeNs at, const BuiltinEvent event, std::function<bool()> locality_guard
) {
    if (!event) throw std::invalid_argument("scheduled built-in callback is empty");
    cancel();
    if (loop_ == nullptr) return 0U;
    builtin_binding_ = BuiltinBinding{this, event};
    id_ = loop_->scheduleOwnerLocalBuiltinAt(
        at, {&ScheduledEvent::builtinTrampoline, &builtin_binding_},
        std::move(locality_guard));
    return id_;
}

void ScheduledEvent::cancel() noexcept {
    if (id_ != 0U && loop_ != nullptr) {
        static_cast<void>(loop_->cancel(id_));
    }
    id_ = 0U;
}

} // namespace fil::sim
