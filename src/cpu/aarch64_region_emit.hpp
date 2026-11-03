// Included inside fil::cpu after the AArch64 integer and native emitters.
namespace aarch64detail {

class RegionEmitter : public NativeEmitter {
public:
    void movW(std::uint32_t rd, std::uint32_t rn) { logicReg(2U, rd, 31U, rn); }
    void addImmX(std::uint32_t rd, std::uint32_t rn, std::uint32_t value) {
        emit(0x91000000U | (value << 10U) | (rn << 5U) | rd);
    }
    void addsX(std::uint32_t rd, std::uint32_t rn, std::uint32_t rm) {
        emit(0xAB000000U | (rm << 16U) | (rn << 5U) | rd);
    }
    void orrX(std::uint32_t rd, std::uint32_t rn, std::uint32_t rm) {
        emit(0xAA000000U | (rm << 16U) | (rn << 5U) | rd);
    }
    void branchReg(std::uint32_t rn) { emit(0xD61F0000U | (rn << 5U)); }
    void callReg(std::uint32_t rn) { emit(0xD63F0000U | (rn << 5U)); }
    void sizedLoad(std::uint32_t rd, std::uint32_t base, std::uint32_t off,
                   std::uint32_t width, bool sign = false) {
        const auto opcode = width == 1U ? (sign ? 0x39C00000U : 0x39400000U)
            : width == 2U ? (sign ? 0x79C00000U : 0x79400000U) : 0xB9400000U;
        emit(opcode | ((off / width) << 10U) | (base << 5U) | rd);
    }
    void sizedStore(std::uint32_t rd, std::uint32_t base, std::uint32_t off, std::uint32_t width) {
        const auto opcode = width == 1U ? 0x39000000U : width == 2U ? 0x79000000U : 0xB9000000U;
        emit(opcode | ((off / width) << 10U) | (base << 5U) | rd);
    }
    void tag(std::uint32_t reg) { immX(16U, std::uint64_t{1U} << 63U); orrX(reg, reg, 16U); }
    void untag(std::uint32_t reg) { ubfm64(reg, reg, 0U, 62U); }
    void mulW(std::uint32_t rd, std::uint32_t rn, std::uint32_t rm) {
        emit(0x1B000000U | (rm << 16U) | (31U << 10U) | (rn << 5U) | rd); // madd wd, wn, wm, wzr
    }
};

// Every window uses this ABI, so linked transfers share the original frame.
// w2..w14 retain guest r0..r12; w15 retains guest LR. Metadata uses x19..x28.
// Guest NZCV lives in w26 across admission/address comparisons and is restored
// to architectural NZCV only while emitting integer operations or predicates.
bool emitRegion(const NativeRegionPlan& plan, const void* resume_table,
                std::uint32_t image_base, std::uint32_t image_size,
                RegionEmitter& e, std::size_t& resume_label, std::string& error) {
    constexpr std::uint32_t ctx = 19U, cpu = 20U, count = 21U, cycles = 22U;
    constexpr std::uint32_t fetch = 23U, dirty = 24U, sp = 25U, nzcv = 26U, pointer = 27U, target = 28U;
    constexpr std::uint32_t frame_bytes = 256U, guest_save = 96U, dynamic_pc = 208U;
    constexpr std::uint32_t writeback = 216U, sp_written = 220U, saved_cycles = 224U, original_address = 232U;
    constexpr auto regs = offsetof(CpuState, r), psr = offsetof(CpuState, xpsr);
    const auto early = e.makeLabel(), reload = e.makeLabel(), publish = e.makeLabel();
    const auto indirect = e.makeLabel(), external = e.makeLabel();
    resume_label = indirect;
    struct Labels { std::size_t entry, instructions, credit, checked; };
    std::array<Labels, NativeRegionPlan::max_ops> labels;
    for (std::size_t i = 0U; i < plan.count; ++i)
        labels[i] = {e.makeLabel(), e.makeLabel(), e.makeLabel(), e.makeLabel()};
    const auto guest = [&](std::uint32_t rd, std::uint8_t reg) {
        e.movW(rd, reg == 13U ? sp : hostReg(reg));
    };
    const auto put_guest = [&](std::uint8_t reg, std::uint32_t source) {
        if (reg == 13U) { e.movW(sp, source); e.storeImm(31U, sp_written, 1U); }
        else e.movW(hostReg(reg), source);
    };
    const auto bank = [&](std::uint32_t source, bool store) {
        const auto msp = e.makeLabel(), ready = e.makeLabel();
        e.load(0U, cpu, psr); e.movImm(16U, xpsr_ipsr_mask); e.logicReg(0U, 0U, 0U, 16U);
        e.cbnzW(0U, msp);
        e.load(0U, cpu, offsetof(CpuState, control)); e.ubfm(0U, 0U, 1U, 1U); e.cbzW(0U, msp);
        if (store) e.store(source, cpu, offsetof(CpuState, psp));
        else e.load(source, cpu, offsetof(CpuState, psp));
        e.b(ready); e.bindLabel(msp);
        if (store) e.store(source, cpu, offsetof(CpuState, msp));
        else e.load(source, cpu, offsetof(CpuState, msp));
        e.bindLabel(ready);
    };
    const auto pack_fetch = [&](std::uint32_t pc_value, std::uint32_t end) {
        e.immX(fetch, (static_cast<std::uint64_t>(pc_value) << 32U) | end);
    };
    const auto exit = [&](NativeRegionExit why, std::uint32_t next_pc) {
        e.movImm(0U, next_pc); e.movImm(1U, static_cast<std::uint32_t>(why)); e.b(publish);
    };
    const auto edge_address = [&](std::uint32_t address) {
        if (resume_table != nullptr) { e.movImm(target, address); e.b(external); }
        else exit(NativeRegionExit::external_edge, address);
    };
    const auto edge = [&](const NativeRegionOp& op, bool taken) {
        const auto index = op.edges[op.flow == RegionFlow::conditional && taken ? 1U : 0U];
        const auto address = taken || op.flow == RegionFlow::jump || op.flow == RegionFlow::call
            ? op.target : op.fallthrough;
        if (index == NativeRegionOp::external) edge_address(address);
        else e.b(labels[index].entry);
    };
    const auto account_mmio = [&] {
        using Effects = mem::MemoryBus::DirectMmioEffects;
        for (const auto member : {offsetof(Effects, mmio_generation), offsetof(Effects, side_effect_generation)}) {
            e.load(16U, ctx, offsetof(NativeRegionContext, mmio_effects) + member, true);
            e.load(0U, 16U, 0U, true); e.addImmX(0U, 0U, 1U); e.store(0U, 16U, 0U, true);
        }
    };
    const auto window = [&](std::size_t binding, std::uint32_t width, bool write, std::size_t miss) {
        using Read = mem::MemoryBus::DirectReadWindow;
        using Write = mem::MemoryBus::DirectWriteWindow;
        e.load(16U, ctx, binding + (write ? offsetof(Write, begin) : offsetof(Read, begin)));
        e.addSubReg(2U, 0U, pointer, 16U);
        e.load(16U, ctx, binding + (write ? offsetof(Write, size) : offsetof(Read, size)));
        e.movImm(17U, width); e.addSubReg(3U, 16U, 16U, 17U); e.cond(3U, miss);
        e.cmpW(0U, 16U); e.cond(8U, miss);
        e.load(16U, ctx, binding + (write ? offsetof(Write, bytes) : offsetof(Read, bytes)), true);
        e.addX(pointer, 16U, 0U);
    };
    const auto mmio = [&](bool write, std::size_t ready, std::size_t miss) {
        for (std::size_t slot = 0U; slot < 2U; ++slot) {
            const auto next = slot == 1U ? miss : e.makeLabel();
            if (write) {
                using Word = mem::MemoryBus::DirectMmioWriteWord;
                using Operation = mem::MmioDevice::DirectWriteWord;
                const auto binding = offsetof(NativeRegionContext, mmio_write) + slot * sizeof(Word);
                e.load(0U, ctx, binding + offsetof(Word, address)); e.cmpW(pointer, 0U); e.cond(1U, next);
                e.load(0U, ctx, binding + offsetof(Word, operation) + offsetof(Operation, execute), true);
                e.cbzX(0U, next); e.addImmX(pointer, ctx, static_cast<std::uint32_t>(binding));
            } else {
                using Word = mem::MemoryBus::DirectMmioReadWord;
                const auto binding = offsetof(NativeRegionContext, mmio_read) + slot * sizeof(Word);
                e.load(0U, ctx, binding + offsetof(Word, address)); e.cmpW(pointer, 0U); e.cond(1U, next);
                e.load(0U, ctx, binding + offsetof(Word, value), true); e.cbzX(0U, next); e.movX(pointer, 0U);
            }
            e.tag(pointer); e.b(ready);
            if (slot == 0U) e.bindLabel(next);
        }
    };
    const auto guard = [&](std::uint32_t width, bool write, bool literal, bool allow_mmio, std::size_t miss) {
        const auto ready = e.makeLabel();
        if (write) {
            const auto slow = allow_mmio ? e.makeLabel() : miss;
            window(offsetof(NativeRegionContext, ram_write), width, true, slow); e.b(ready);
            if (allow_mmio) { e.bindLabel(slow); mmio(true, ready, miss); }
        } else {
            const auto second = e.makeLabel(), slow = allow_mmio ? e.makeLabel() : miss;
            window(literal ? offsetof(NativeRegionContext, code_read) : offsetof(NativeRegionContext, ram_read), width, false, second);
            e.b(ready); e.bindLabel(second);
            window(literal ? offsetof(NativeRegionContext, ram_read) : offsetof(NativeRegionContext, code_read), width, false, slow);
            e.b(ready);
            if (allow_mmio) { e.bindLabel(slow); mmio(false, ready, miss); }
        }
        e.bindLabel(ready);
    };
    const auto scalar_address = [&](const NativeRegionOp& op, std::size_t miss) {
        const auto& inst = op.instruction;
        if (inst.absolute) e.movImm(pointer, inst.absolute_address);
        else {
            guest(pointer, inst.rn);
            if (!op.address.index) e.store(pointer, 31U, original_address);
            if (op.address.register_offset) {
                guest(16U, op.address.rm);
                if (op.address.shift != 0U) e.ubfm(16U, 16U, 32U - op.address.shift, 31U - op.address.shift);
                e.addSubReg(op.address.add ? 0U : 2U, pointer, pointer, 16U);
            } else if (inst.offset != 0U) e.addSubImm(op.address.add ? 0U : 2U, pointer, pointer, inst.offset);
            if (op.address.writeback) e.store(pointer, 31U, writeback);
            if (!op.address.index) e.load(pointer, 31U, original_address);
        }
        const bool write = !nativeOpIsLoad(inst.kind);
        const bool word = nativeOpWidth(inst.kind) == 4U && !inst.absolute && inst.rn != 13U;
        guard(nativeOpWidth(inst.kind), write, inst.absolute, word, miss);
    };
    const auto store_ram = [&](std::uint8_t reg, std::uint32_t displacement, std::uint32_t width) {
        guest(0U, reg); e.sizedLoad(16U, pointer, displacement, width);
        e.sizedStore(0U, pointer, displacement, width); e.logicReg(3U, 16U, 16U, 0U);
        for (std::uint32_t byte = 0U; byte < width; ++byte) {
            e.ubfm(17U, 16U, byte * 8U, byte * 8U + 7U); e.cmpImm(17U, 0U);
            e.emit(0x1A9F07E0U | 17U); // cset w17, ne
            e.addX(dirty, dirty, 17U);
        }
    };
    const auto scalar_body = [&](const NativeRegionOp& op, std::size_t miss) {
        const auto& inst = op.instruction;
        const auto width = nativeOpWidth(inst.kind);
        const bool tagged = width == 4U && !inst.absolute && inst.rn != 13U;
        if (nativeOpIsLoad(inst.kind)) {
            if (tagged) {
                const auto ordinary = e.makeLabel(); e.lsr64(0U, pointer, 63U); e.cbzX(0U, ordinary);
                e.untag(pointer); account_mmio(); e.bindLabel(ordinary);
            }
            const bool sign = inst.kind == NativeOp::load_byte_signed || inst.kind == NativeOp::load_half_signed;
            e.sizedLoad(hostReg(inst.rd), pointer, 0U, width, sign);
        } else if (tagged) {
            const auto ordinary = e.makeLabel(), committed = e.makeLabel(), declined = e.makeLabel();
            e.lsr64(0U, pointer, 63U); e.cbzX(0U, ordinary); e.untag(pointer);
            for (std::uint32_t reg = 2U; reg <= 15U; ++reg) e.store(reg, 31U, guest_save + (reg - 2U) * 8U, true);
            using Word = mem::MemoryBus::DirectMmioWriteWord;
            using Operation = mem::MmioDevice::DirectWriteWord;
            guest(1U, inst.rd);
            e.load(0U, pointer, offsetof(Word, operation) + offsetof(Operation, context), true);
            e.load(16U, pointer, offsetof(Word, operation) + offsetof(Operation, execute), true); e.callReg(16U);
            e.andImm(0U, 0U, 0U, 7U);
            for (std::uint32_t reg = 2U; reg <= 15U; ++reg) e.load(reg, 31U, guest_save + (reg - 2U) * 8U, true);
            e.cbzW(0U, declined); account_mmio(); e.b(committed);
            e.bindLabel(declined); e.load(cycles, 31U, saved_cycles, true); e.b(miss);
            e.bindLabel(ordinary); store_ram(inst.rd, 0U, 4U); e.bindLabel(committed);
        } else store_ram(inst.rd, 0U, width);
        if (op.address.writeback) { e.load(0U, 31U, writeback); put_guest(inst.rn, 0U); }
    };
    const auto stack_address = [&](const NativeRegionOp& op, std::size_t miss) {
        const auto bytes = op.stack.count() * 4U;
        e.movW(pointer, sp);
        if (op.stack.push()) {
            e.movImm(0U, bytes); e.cmpW(pointer, 0U); e.cond(3U, miss);
            e.addSubImm(2U, pointer, pointer, bytes);
        }
        guard(bytes, op.stack.push(), false, false, miss);
        if (op.stack.loadsPc()) {
            e.load(0U, pointer, bytes - 4U); e.andImm(16U, 0U, 0U, 0U); e.cbzW(16U, miss);
            e.movImm(16U, 0xffffffe0U); e.logicReg(0U, 17U, 0U, 16U); e.cmpW(17U, 16U); e.cond(0U, miss);
            e.ubfm(0U, 0U, 1U, 31U); e.addSubReg(0U, 0U, 0U, 0U); e.store(0U, 31U, dynamic_pc);
        }
    };
    const auto stack_body = [&](const NativeRegionOp& op) {
        e.addSubImm(op.stack.push() ? 2U : 0U, sp, sp, op.stack.count() * 4U);
        e.storeImm(31U, sp_written, 1U);
        for (std::uint32_t i = 0U; i < op.stack.count(); ++i) {
            const auto reg = static_cast<std::uint8_t>(op.stack.reg(i));
            if (op.stack.push()) store_ram(reg, i * 4U, 4U);
            else if (reg != 15U) e.load(hostReg(reg), pointer, i * 4U);
        }
    };
    const auto charge = [&](const NativeRegionOp& op, std::uint32_t pipeline, std::uint32_t extra_sequential,
                            std::size_t insufficient) {
        const auto sequential = e.makeLabel(), ready = e.makeLabel();
        e.movImm(16U, op.pc); e.cmpW(fetch, 16U); e.cond(0U, sequential);
        e.load(0U, ctx, offsetof(NativeRegionContext, nonsequential_fetch_stall)); e.b(ready);
        e.bindLabel(sequential); e.load(0U, ctx, offsetof(NativeRegionContext, sequential_fetch_stall));
        e.bindLabel(ready);
        if (extra_sequential != 0U) {
            e.load(16U, ctx, offsetof(NativeRegionContext, sequential_fetch_stall));
            for (std::uint32_t i = 0U; i < extra_sequential; ++i) e.addX(0U, 0U, 16U);
        }
        e.movImm(16U, pipeline); e.addX(0U, 0U, 16U);
        e.addsX(target, cycles, 0U); e.cond(2U, insufficient);
        e.load(0U, ctx, offsetof(NativeRegionContext, cycle_limit), true); e.cmp64(target, 0U); e.cond(8U, insufficient);
    };
    const auto body = [&](const NativeRegionOp& op, std::size_t index, bool taken) {
        e.store(cycles, 31U, saved_cycles, true);
        charge(op, taken ? op.taken_cycles : op.fallthrough_cycles, 0U, labels[index].credit);
        e.movX(cycles, target);
        if (op.stack.count() != 0U) stack_body(op);
        else if (op.flow == RegionFlow::next) {
            if (nativeOpIsMemory(op.instruction.kind)) scalar_body(op, labels[index].checked);
            else {
                e.msrNzcv(nzcv);
                if (!emitOp(e, op.instruction.integer, error)) return false;
                e.mrsNzcv(nzcv);
            }
        } else if (op.flow == RegionFlow::call || op.instruction.kind == NativeOp::blx_reg)
            e.movImm(hostReg(14U), op.fallthrough | 1U);
        e.addSubImm(0U, count, count, 1U); pack_fetch(op.pc, op.fallthrough);
        if (op.flow == RegionFlow::indirect) { e.load(target, 31U, dynamic_pc); e.b(indirect); }
        else edge(op, taken);
        return true;
    };
    // Straight-line integer runs admit one budget and charge the whole run, so
    // the per-instruction entry check, fetch select, and cycle compare collapse
    // to a single block. Guest NZCV stays live in the host flags across the run,
    // which removes the per-op msr/mrs round trip. Interior operations keep
    // their own entries for indirect dispatch and partial cuts.
    constexpr std::size_t max_span_ops = 16U;
    struct LinearSpan {
        std::array<std::uint16_t, max_span_ops> ops{};
        std::uint16_t length{0U};
        std::uint32_t pipeline_cycles{0U};
    };
    std::array<LinearSpan, NativeRegionPlan::max_ops> spans{};
    std::bitset<NativeRegionPlan::max_ops> covered;
    for (std::uint16_t start = 0U; start < plan.count; ++start) {
        if (covered[start] || plan.ops[start].predicated_pair.valid) continue;
        LinearSpan span;
        std::bitset<NativeRegionPlan::max_ops> seen;
        auto cursor = start;
        while (cursor != NativeRegionOp::external && span.length < span.ops.size()
            && !covered[cursor] && !seen[cursor]) {
            const auto& op = plan.ops[cursor];
            if (op.flow != RegionFlow::next || op.stack.count() != 0U
                || op.instruction.kind != NativeOp::integer || op.predicated_pair.valid) break;
            seen.set(cursor);
            span.ops[span.length++] = cursor;
            span.pipeline_cycles += op.fallthrough_cycles;
            const auto next = op.edges[0];
            if (next != NativeRegionOp::external && plan.ops[next].pc != op.fallthrough) break;
            cursor = next;
        }
        if (span.length < 3U) continue;
        spans[start] = span;
        for (std::size_t n = 0U; n < span.length; ++n) covered.set(span.ops[n]);
    }
    const auto span_charge = [&](const NativeRegionOp& first, std::uint32_t length,
                                 std::uint32_t pipeline, std::size_t insufficient) {
        const auto sequential = e.makeLabel(), ready = e.makeLabel();
        e.movImm(16U, first.pc); e.cmpW(fetch, 16U); e.cond(0U, sequential);
        e.load(0U, ctx, offsetof(NativeRegionContext, nonsequential_fetch_stall));
        e.b(ready);
        e.bindLabel(sequential);
        e.load(0U, ctx, offsetof(NativeRegionContext, sequential_fetch_stall));
        e.bindLabel(ready);
        e.load(16U, ctx, offsetof(NativeRegionContext, sequential_fetch_stall));
        e.movImm(17U, length - 1U); e.mulW(16U, 16U, 17U);
        e.addSubReg(0U, 0U, 0U, 16U);
        e.movImm(16U, pipeline); e.addSubReg(0U, 0U, 0U, 16U);
        e.addsX(target, cycles, 0U); e.cond(2U, insufficient);
        e.load(0U, ctx, offsetof(NativeRegionContext, cycle_limit), true);
        e.cmp64(target, 0U); e.cond(8U, insufficient);
    };
    const auto linear_span = [&](const LinearSpan& span) {
        const auto slow = e.makeLabel();
        const auto& first = plan.ops[span.ops[0]];
        const auto& last = plan.ops[span.ops[span.length - 1U]];
        e.load(0U, ctx, offsetof(NativeRegionContext, instruction_limit));
        e.addSubReg(2U, 0U, 0U, count); e.cmpImm(0U, span.length); e.cond(3U, slow);
        span_charge(first, span.length, span.pipeline_cycles, slow);
        e.movX(cycles, target);
        e.msrNzcv(nzcv);
        for (std::size_t n = 0U; n < span.length; ++n) {
            if (!emitOp(e, plan.ops[span.ops[n]].instruction.integer, error)) return false;
        }
        e.mrsNzcv(nzcv);
        e.addSubImm(0U, count, count, span.length);
        pack_fetch(last.pc, last.fallthrough);
        edge(last, false);
        e.bindLabel(slow);
        return true;
    };

    e.sub64Imm(31U, 31U, frame_bytes);
    for (std::uint32_t reg = 19U; reg <= 30U; ++reg) e.store(reg, 31U, (reg - 19U) * 8U, true);
    e.movX(ctx, 0U); e.call(reinterpret_cast<std::uintptr_t>(&nativeRegionEntryValid));
    e.andImm(0U, 0U, 0U, 7U); e.cbzW(0U, early);
    e.bindLabel(reload);
    e.load(cpu, ctx, offsetof(NativeRegionContext, cpu_state), true);
    for (std::uint32_t reg = 0U; reg < 13U; ++reg) e.load(hostReg(static_cast<std::uint8_t>(reg)), cpu, regs + reg * 4U);
    e.load(hostReg(14U), cpu, regs + 14U * 4U);
    bank(sp, false); e.storeImm(31U, sp_written, 0U);
    e.load(nzcv, cpu, psr); e.ubfm(nzcv, nzcv, 28U, 31U); e.ubfm(nzcv, nzcv, 4U, 3U);
    e.movImm(dirty, 0U); e.load(count, ctx, offsetof(NativeRegionContext, retired_count));
    e.load(cycles, ctx, offsetof(NativeRegionContext, retired_cycles), true);
    e.load(fetch, ctx, offsetof(NativeRegionContext, fetch_end));
    const auto fetch_ready = e.makeLabel();
    e.sizedLoad(0U, ctx, offsetof(NativeRegionContext, have_fetch), 1U); e.cbnzW(0U, fetch_ready);
    e.movImm(fetch, plan.entry_pc | 1U);
    e.bindLabel(fetch_ready); e.load(0U, cpu, offsetof(CpuState, instruction_address)); e.bfi64(fetch, 0U, 32U, 32U);
    e.load(target, cpu, regs + 15U * 4U); e.b(indirect);

    for (std::size_t i = 0U; i < plan.count; ++i) {
        const auto& op = plan.ops[i];
        e.bindLabel(labels[i].entry);
        e.load(0U, ctx, offsetof(NativeRegionContext, instruction_limit)); e.cmpW(count, 0U); e.cond(2U, labels[i].instructions);
        if (spans[i].length != 0U && !linear_span(spans[i])) return false;
        if (op.predicated_pair.valid) {
            const auto slow = e.makeLabel(), then_path = e.makeLabel(), committed = e.makeLabel();
            e.addSubReg(2U, 0U, 0U, count); e.cmpImm(0U, 3U); e.cond(3U, slow);
            charge(op, 3U, 2U, slow); e.movX(cycles, target);
            e.msrNzcv(nzcv); e.cond(static_cast<std::uint32_t>(op.predicated_pair.condition), then_path);
            e.movImm(hostReg(op.predicated_pair.registers[1]), op.predicated_pair.values[1]); e.b(committed);
            e.bindLabel(then_path); e.movImm(hostReg(op.predicated_pair.registers[0]), op.predicated_pair.values[0]);
            e.bindLabel(committed); e.load(0U, cpu, psr); e.movImm(16U, ~xpsr_it_mask); e.logicReg(0U, 0U, 0U, 16U); e.store(0U, cpu, psr);
            e.addSubImm(0U, count, count, 3U); pack_fetch(op.pc + 4U, op.pc + 6U);
            if (op.edges[0] != NativeRegionOp::external) e.b(labels[op.edges[0]].entry);
            else edge_address(op.pc + 6U);
            e.bindLabel(slow);
        }
        if (op.stack.count() != 0U) {
            stack_address(op, labels[i].checked);
            if (op.stack.loadsPc()) {
                const auto sequential = e.makeLabel();
                e.load(0U, 31U, dynamic_pc); e.movImm(16U, op.fallthrough); e.cmpW(0U, 16U); e.cond(0U, sequential);
                if (!body(op, i, true)) return false;
                e.bindLabel(sequential);
            }
            if (!body(op, i, false)) return false;
        } else if (op.flow == RegionFlow::indirect) {
            guest(0U, op.instruction.kind == NativeOp::bx_lr ? 14U : op.instruction.rn);
            e.andImm(16U, 0U, 0U, 0U); e.cbzW(16U, labels[i].checked);
            e.movImm(16U, 0xffffffe0U); e.logicReg(0U, 17U, 0U, 16U); e.cmpW(17U, 16U); e.cond(0U, labels[i].checked);
            e.ubfm(0U, 0U, 1U, 31U); e.addSubReg(0U, 0U, 0U, 0U); e.store(0U, 31U, dynamic_pc);
            const auto sequential = e.makeLabel(); e.movImm(16U, op.fallthrough); e.cmpW(0U, 16U); e.cond(0U, sequential);
            if (!body(op, i, true)) return false;
            e.bindLabel(sequential); if (!body(op, i, false)) return false;
        } else if (op.flow == RegionFlow::checked) e.b(labels[i].checked);
        else if (nativeOpIsMemory(op.instruction.kind)) {
            scalar_address(op, labels[i].checked); if (!body(op, i, false)) return false;
        } else if (op.flow == RegionFlow::conditional) {
            const auto taken = e.makeLabel(); const auto& inst = op.instruction.integer;
            if (inst.op == MicroOp::cbz) e.cbzW(hostReg(inst.rn), taken);
            else if (inst.op == MicroOp::cbnz) e.cbnzW(hostReg(inst.rn), taken);
            else { e.msrNzcv(nzcv); e.cond(static_cast<std::uint32_t>(inst.condition), taken); }
            if (!body(op, i, false)) return false;
            e.bindLabel(taken); if (!body(op, i, true)) return false;
        } else if (!body(op, i, op.flow != RegionFlow::next)) return false;
    }
    e.bindLabel(indirect);
    std::array<std::uint16_t, NativeRegionPlan::max_ops> order;
    for (std::uint16_t i = 0U; i < plan.count; ++i) order[i] = i;
    std::sort(order.begin(), order.begin() + plan.count, [&](auto a, auto b) { return plan.ops[a].pc < plan.ops[b].pc; });
    const auto dispatch = [&](auto&& self, std::size_t begin, std::size_t end) -> void {
        if (begin == end) { e.b(external); return; }
        const auto middle = begin + (end - begin) / 2U;
        e.movImm(0U, plan.ops[order[middle]].pc); e.cmpW(target, 0U); e.cond(0U, labels[order[middle]].entry);
        if (end - begin == 1U) { e.b(external); return; }
        const auto upper = e.makeLabel(); e.cond(8U, upper);
        self(self, begin, middle); e.bindLabel(upper); self(self, middle + 1U, end);
    };
    dispatch(dispatch, 0U, plan.count);
    e.bindLabel(external);
    if (resume_table != nullptr) {
        const auto outside = e.makeLabel();
        e.movImm(16U, image_base); e.addSubReg(2U, 0U, target, 16U);
        e.movImm(16U, image_size); e.cmpW(0U, 16U); e.cond(2U, outside);
        e.andImm(16U, 0U, 0U, 0U); e.cbnzW(16U, outside);
        e.ubfm(0U, 0U, 9U, 31U); e.ubfm64(0U, 0U, 61U, 60U);
        e.immX(16U, reinterpret_cast<std::uintptr_t>(resume_table)); e.addX(0U, 0U, 16U); e.load(0U, 0U, 0U, true); e.branchReg(0U);
        e.bindLabel(outside);
    }
    e.movW(0U, target); e.movImm(1U, static_cast<std::uint32_t>(NativeRegionExit::external_edge)); e.b(publish);
    for (std::size_t i = 0U; i < plan.count; ++i) {
        e.bindLabel(labels[i].instructions); exit(NativeRegionExit::instruction_budget, plan.ops[i].pc);
        e.bindLabel(labels[i].credit); exit(NativeRegionExit::cycle_budget, plan.ops[i].pc);
        e.bindLabel(labels[i].checked); exit(NativeRegionExit::checked_instruction, plan.ops[i].pc);
    }
    e.bindLabel(publish);
    e.store(0U, cpu, regs + 15U * 4U); e.store(1U, ctx, offsetof(NativeRegionContext, exit_reason));
    e.store(count, ctx, offsetof(NativeRegionContext, retired_count)); e.store(cycles, ctx, offsetof(NativeRegionContext, retired_cycles), true);
    for (std::uint32_t reg = 0U; reg < 13U; ++reg) e.store(hostReg(static_cast<std::uint8_t>(reg)), cpu, regs + reg * 4U);
    e.store(hostReg(14U), cpu, regs + 14U * 4U);
    const auto stack_published = e.makeLabel(); e.load(0U, 31U, sp_written); e.cbzW(0U, stack_published);
    e.store(sp, cpu, regs + 13U * 4U); bank(sp, true); e.bindLabel(stack_published);
    const auto effects_published = e.makeLabel(); e.cbzX(dirty, effects_published);
    using Write = mem::MemoryBus::DirectWriteWindow;
    for (const auto member : {offsetof(Write, mutation_sequence), offsetof(Write, side_effect_generation)}) {
        e.load(16U, ctx, offsetof(NativeRegionContext, ram_write) + member, true);
        e.load(0U, 16U, 0U, true); e.addX(0U, 0U, dirty); e.store(0U, 16U, 0U, true);
    }
    e.bindLabel(effects_published);
    e.load(0U, cpu, psr); e.clearTopNibble(0U); e.logicReg(2U, 0U, 0U, nzcv); e.store(0U, cpu, psr);
    const auto counters_published = e.makeLabel(); e.cbzW(count, counters_published);
    e.store(fetch, ctx, offsetof(NativeRegionContext, fetch_end)); e.movImm(0U, 1U); e.sizedStore(0U, ctx, offsetof(NativeRegionContext, have_fetch), 1U);
    e.lsr64(0U, fetch, 32U); e.store(0U, cpu, offsetof(CpuState, instruction_address));
    e.bindLabel(counters_published);
    e.load(0U, ctx, offsetof(NativeRegionContext, exit_reason)); e.cmpImm(0U, static_cast<std::uint32_t>(NativeRegionExit::checked_instruction)); e.cond(1U, early);
    e.movX(0U, ctx); e.call(reinterpret_cast<std::uintptr_t>(&nativeRegionContinue)); e.andImm(0U, 0U, 0U, 7U); e.cbnzW(0U, reload);
    e.bindLabel(early);
    for (std::uint32_t reg = 19U; reg <= 30U; ++reg) e.load(reg, 31U, (reg - 19U) * 8U, true);
    e.addImmX(31U, 31U, frame_bytes); e.ret();
    return e.patchBranches(error);
}
} // namespace aarch64detail
