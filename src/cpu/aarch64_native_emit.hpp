// Included inside fil::cpu, after the pure AArch64 stencil emitter.
// Native blocks and traces share one per-instruction execution/commit model.
namespace aarch64detail {

class NativeEmitter : public Emitter {
public:
    void movX(std::uint32_t rd, std::uint32_t rn) {
        emit(0xAA0003E0U | (rn << 16U) | rd);
    }
    void immX(std::uint32_t rd, std::uint64_t value) {
        emit(0xD2800000U | ((static_cast<std::uint32_t>(value) & 0xffffU) << 5U) | rd);
        for (unsigned i = 1; i < 4; ++i)
            emit(0xF2800000U | (i << 21U)
                | ((static_cast<std::uint32_t>(value >> (16U * i)) & 0xffffU) << 5U) | rd);
    }
    void load(std::uint32_t rd, std::uint32_t base, std::size_t off, bool wide = false) {
        emit((wide ? 0xF9400000U : 0xB9400000U)
            | (static_cast<std::uint32_t>(off / (wide ? 8U : 4U)) << 10U) | (base << 5U) | rd);
    }
    void store(std::uint32_t rd, std::uint32_t base, std::size_t off, bool wide = false) {
        emit((wide ? 0xF9000000U : 0xB9000000U)
            | (static_cast<std::uint32_t>(off / (wide ? 8U : 4U)) << 10U) | (base << 5U) | rd);
    }
    void storeImm(std::uint32_t base, std::size_t off, std::uint32_t value) {
        movImm(17U, value); store(17U, base, off);
    }
    void cbzX(std::uint32_t rt, std::size_t label) {
        fixups.push_back({words.size(), label, 1U}); emit(0xB4000000U | rt);
    }
    void cond(std::uint32_t condition, std::size_t label) {
        fixups.push_back({words.size(), label, 1U}); emit(0x54000000U | condition);
    }
    void cmpW(std::uint32_t rn, std::uint32_t rm) { addSubReg(3U, 31U, rn, rm); }
    void addX(std::uint32_t rd, std::uint32_t rn, std::uint32_t rm) {
        emit(0x8B000000U | (rm << 16U) | (rn << 5U) | rd);
    }
    void subX(std::uint32_t rd, std::uint32_t rn, std::uint32_t rm) {
        emit(0xCB000000U | (rm << 16U) | (rn << 5U) | rd);
    }
    void call(std::uintptr_t address) {
        immX(16U, address); emit(0xD63F0200U); // blr x16
    }
};

struct Op {
    NativeInst native;
    std::uint32_t next_pc;
    bool trace{false};
    bool expected_taken{false};
    bool stack{false};
    bool push{false};
    std::uint32_t register_list{0};
};

bool emitNative(std::span<const Op> ops, std::vector<std::uint8_t>& code, std::string& error) {
    NativeEmitter e;
    constexpr std::uint32_t ctx = 19U, cpu = 20U, cost = 21U;
    constexpr auto r = offsetof(CpuState, r);
    constexpr auto psr = offsetof(CpuState, xpsr);
    constexpr auto retired = offsetof(NativeExecutionContext, retired_count);
    constexpr auto cycles = offsetof(NativeExecutionContext, retired_cycles);
    constexpr auto reason = offsetof(NativeExecutionContext, exit_reason);
    constexpr auto failing = offsetof(NativeExecutionContext, failing_instruction_pc);
    const auto early = e.makeLabel(), done = e.makeLabel();
    e.cbzX(0U, early);
    e.load(9U, 0U, offsetof(NativeExecutionContext, cpu_state), true);
    e.cbzX(9U, early);
    // x19=context, x20=CPU, x21=current cost survive all scalar helpers.
    // 48 bytes of nonvolatile saves plus 16 bytes for load output/padding.
    e.sub64Imm(31U, 31U, 64U);
    e.store(19U, 31U, 0U, true); e.store(20U, 31U, 8U, true);
    e.store(21U, 31U, 16U, true); e.store(30U, 31U, 24U, true);
    e.store(22U, 31U, 32U, true); e.store(23U, 31U, 40U, true);
    e.movX(ctx, 0U); e.movX(cpu, 9U);
    e.movImm(22U, 0U); // x22 = 0 checked, 1 validated admission.
    const auto checked_table = e.makeLabel();
    e.load(9U, ctx, offsetof(NativeExecutionContext, admitted_cycle_prefix), true);
    e.cbzX(9U, checked_table); // Null hints retain the original no-helper path.
    e.movX(0U, ctx); e.call(reinterpret_cast<std::uintptr_t>(&nativeAdmittedCyclePrefixValid));
    e.andImm(0U, 0U, 0U, 7U); // Only the bool result byte is defined by the ABI.
    e.cmpImm(0U, 0U); // Normalize the scalar bool to 0/1 in x22.
    e.emit(0x9A9F07F6U); // csinc x22, xzr, xzr, eq (cset x22, ne)
    e.cbzX(22U, checked_table);
    e.load(23U, ctx, offsetof(NativeExecutionContext, admitted_cycle_prefix), true);
    e.b(checked_table);
    e.bindLabel(checked_table);
    e.load(21U, ctx, offsetof(NativeExecutionContext, charged_cycles), true);
    e.store(31U, ctx, retired); e.store(31U, ctx, reason);

    for (std::size_t i = 0; i < ops.size(); ++i) {
        const auto& op = ops[i];
        const auto& m = op.native;
        const auto& inst = m.integer;
        const auto instruction_exit = e.makeLabel(), cycle_exit = e.makeLabel();
        const auto unsupported_exit = e.makeLabel(), helper_exit = e.makeLabel();
        const auto mismatch_exit = e.makeLabel(), next = e.makeLabel();
        const auto dynamic_return_exit = e.makeLabel();
        e.storeImm(ctx, offsetof(NativeExecutionContext, current_instruction_pc), m.pc);
        e.load(9U, ctx, offsetof(NativeExecutionContext, instruction_limit));
        e.cmpImm(9U, static_cast<std::uint32_t>(i + 1U)); e.cond(3U, instruction_exit); // lo
        // Stop before the next op after a committed-span MMIO, so the retired
        // prefix already includes the MMIO instruction. Mask the bool byte:
        // a word load also sees the field's uninitialized alignment padding.
        e.load(9U, ctx, offsetof(NativeExecutionContext, boundary_stop));
        e.andImm(9U, 9U, 0U, 7U);
        e.cbnzW(9U, instruction_exit);
        // Both admission modes need the per-instruction charge table. Reload
        // it here: integer stencils and scalar helpers may clobber x11.
        e.load(11U, ctx, offsetof(NativeExecutionContext, charged_cycles), true);
        e.cbzX(11U, unsupported_exit);
        e.load(9U, ctx, offsetof(NativeExecutionContext, charged_cycles_count));
        e.cmpImm(9U, static_cast<std::uint32_t>(i + 1U)); e.cond(3U, unsupported_exit);

        const bool blx_reg = m.kind == NativeOp::blx_reg;
        const bool compare_branch = m.kind == NativeOp::integer
            && (inst.op == MicroOp::cbz || inst.op == MicroOp::cbnz);
        const bool branch = blx_reg || compare_branch || (m.kind == NativeOp::integer
            && (inst.op == MicroOp::b || inst.op == MicroOp::bl || inst.op == MicroOp::b_cond));
        if (op.trace && compare_branch) {
            e.load(9U, cpu, r + inst.rn * 4U);
            const bool expect_zero = op.expected_taken == (inst.op == MicroOp::cbz);
            if (expect_zero) e.cbnzW(9U, mismatch_exit);
            else e.cbzW(9U, mismatch_exit);
        }
        if (op.trace && branch && inst.op == MicroOp::b_cond) {
            // ARM and AArch64 use identical condition encodings (AL/NV handled explicitly).
            if (inst.condition == Condition::al) {
                if (!op.expected_taken) e.b(mismatch_exit);
            } else if (inst.condition == Condition::nv) {
                error = "native-aarch64: NV condition"; return false;
            } else {
                e.load(9U, cpu, psr); e.msrNzcv(9U);
                const auto condition = static_cast<std::uint32_t>(inst.condition);
                e.cond(op.expected_taken ? (condition ^ 1U) : condition, mismatch_exit);
            }
        }
        if (blx_reg) {
            e.load(9U, cpu, r + m.rn * 4U);
            e.movImm(10U, op.next_pc | 1U); e.cmpW(9U, 10U); e.cond(1U, mismatch_exit);
            e.movImm(10U, 0xfffffff0U); e.logicReg(0U, 9U, 9U, 10U);
            e.cmpW(9U, 10U); e.cond(0U, mismatch_exit);
        }
        if (m.kind == NativeOp::bx_lr) {
            e.load(9U, cpu, r + 56U);
            e.movImm(10U, op.next_pc | 1U);
            e.cmpW(9U, 10U); e.cond(1U, mismatch_exit);
            // Equality above enforces Thumb bit; exclude EXC_RETURN even if the
            // recorded edge itself happens to name an exception-return token.
            e.movImm(10U, 0xfffffff0U); e.logicReg(0U, 9U, 9U, 10U);
            e.cmpW(9U, 10U); e.cond(0U, mismatch_exit);
        }
        // Subtract before adding to avoid wrapping the cycle accumulator.
        // Admitted mode skips only this cycle-budget probe; branch/indirect
        // guards above still run so a changed edge declines as in checked mode.
        const auto after_cycle_admission = e.makeLabel();
        e.cbnzW(22U, after_cycle_admission);
        e.load(9U, ctx, cycles, true);
        e.load(10U, ctx, offsetof(NativeExecutionContext, cycle_limit), true);
        e.cmp64(9U, 10U); e.cond(8U, cycle_exit); // hi
        e.subX(10U, 10U, 9U);
        e.load(cost, 11U, i * sizeof(std::uint32_t));
        e.cmp64(cost, 10U); e.cond(8U, cycle_exit);
        e.bindLabel(after_cycle_admission);

        if (op.stack) {
            e.movX(0U, ctx); e.movImm(1U, m.pc);
            e.movImm(2U, op.register_list); e.movImm(3U, op.push ? 1U : 0U);
            e.movImm(4U, op.next_pc);
            e.call(reinterpret_cast<std::uintptr_t>(&nativeStackTransfer));
            if (!op.push && (op.register_list & 0x8000U)) {
                e.cmpImm(0U, static_cast<std::uint32_t>(NativeStackResult::dynamic_return));
                e.cond(0U, dynamic_return_exit);
            }
            e.cmpImm(0U, static_cast<std::uint32_t>(NativeStackResult::branch_mismatch));
            e.cond(0U, mismatch_exit);
            e.cbnzW(0U, helper_exit);
        } else if (nativeOpIsMemory(m.kind)) {
            // A committed-span MMIO helper needs the exact prefix cycle count.
            // Checked mode already accumulated it in `cycles`; admitted kernels
            // otherwise defer cumulative cycle publication to their exit.
            const auto have_prefix = e.makeLabel();
            e.cbzW(22U, have_prefix);
            e.load(9U, 23U, i * sizeof(std::uint32_t));
            e.store(9U, ctx, cycles, true);
            e.bindLabel(have_prefix);
            if (m.absolute) e.movImm(2U, m.absolute_address);
            else {
                if (m.rn == 13U) {
                    // SP reads select MSP/PSP, not the possibly stale r[13] mirror.
                    const auto have_sp = e.makeLabel();
                    e.load(2U, cpu, offsetof(CpuState, msp));
                    e.load(9U, cpu, psr); e.andImm(9U, 9U, 0U, 8U);
                    e.cbnzW(9U, have_sp); // handler mode always uses MSP
                    e.load(9U, cpu, offsetof(CpuState, control));
                    e.ubfm(9U, 9U, 1U, 1U); e.cbzW(9U, have_sp);
                    e.load(2U, cpu, offsetof(CpuState, psp)); e.bindLabel(have_sp);
                } else e.load(2U, cpu, r + m.rn * 4U);
                e.addSubImm(0U, 2U, 2U, m.offset);
            }
            const bool load = nativeOpIsLoad(m.kind);
            const auto width = nativeOpWidth(m.kind);
            e.movX(0U, ctx); e.movImm(1U, m.pc);
            if (load) e.emit(0x9100C3E3U); // add x3, sp, #48 (output)
            else e.load(3U, cpu, r + m.rd * 4U);
            const auto helper = width == 1U
                ? (load ? reinterpret_cast<std::uintptr_t>(&nativeRead8) : reinterpret_cast<std::uintptr_t>(&nativeWrite8))
                : width == 2U
                ? (load ? reinterpret_cast<std::uintptr_t>(&nativeRead16) : reinterpret_cast<std::uintptr_t>(&nativeWrite16))
                : (load ? reinterpret_cast<std::uintptr_t>(&nativeRead32) : reinterpret_cast<std::uintptr_t>(&nativeWrite32));
            e.call(helper);
            // Only the bool result byte is guaranteed by the scalar ABI.
            e.andImm(0U, 0U, 0U, 7U); e.cbzW(0U, helper_exit);
            if (load) {
                e.load(9U, 31U, 48U);
                if (m.kind == NativeOp::load_byte_signed) e.sbfm(9U, 9U, 0U, 7U);
                if (m.kind == NativeOp::load_half_signed) e.sbfm(9U, 9U, 0U, 15U);
                e.store(9U, cpu, r + m.rd * 4U);
            }
        } else if (branch) {
            if (inst.op == MicroOp::bl || blx_reg)
                e.storeImm(cpu, r + 56U, (m.pc + m.size) | 1U);
        } else if (m.kind == NativeOp::integer) {
            // Reuse exactly the same integer stencils as the pure backend.
            // Guest flags are materialized at every commit/helper boundary;
            // admission checks and helper calls may freely clobber host NZCV.
            e.movX(0U, cpu);
            for (std::uint8_t reg = 0; reg <= 12U; ++reg) e.ldrW(hostReg(reg), r + reg * 4U);
            e.ldrW(15U, r + 56U); e.ldrW(16U, psr);
            e.msrNzcv(16U); e.clearTopNibble(16U);
            if (!emitOp(e, inst, error)) return false;
            for (std::uint8_t reg = 0; reg <= 12U; ++reg) e.strW(hostReg(reg), r + reg * 4U);
            e.strW(15U, r + 56U);
            e.mrsNzcv(17U); e.logicReg(2U, 16U, 16U, 17U); e.strW(16U, psr);
        }
        e.storeImm(cpu, r + 60U, op.next_pc);
        e.storeImm(cpu, offsetof(CpuState, instruction_address), m.pc);
        const auto skip_cycle_commit = e.makeLabel();
        e.cbnzW(22U, skip_cycle_commit);
        e.load(9U, ctx, cycles, true); e.addX(9U, 9U, cost); e.store(9U, ctx, cycles, true);
        e.bindLabel(skip_cycle_commit);
        e.storeImm(ctx, retired, static_cast<std::uint32_t>(i + 1U));
        e.b(next);
        const auto exit = [&](std::size_t label, NativeExitReason why, bool failed, std::size_t retired_prefix) {
            e.bindLabel(label);
            if (failed) e.storeImm(ctx, failing, m.pc);
            const bool dynamic_return = why == NativeExitReason::dynamic_return;
            if (dynamic_return) {
                e.storeImm(cpu, offsetof(CpuState, instruction_address), m.pc);
                e.storeImm(ctx, retired, static_cast<std::uint32_t>(retired_prefix));
            }
            const auto checked_cycles = e.makeLabel(), cycles_done = e.makeLabel();
            e.cbzX(22U, checked_cycles);
            e.load(9U, 23U, retired_prefix * sizeof(std::uint32_t));
            e.store(9U, ctx, cycles, true); e.b(cycles_done);
            e.bindLabel(checked_cycles);
            if (dynamic_return) {
                e.load(9U, ctx, cycles, true); e.addX(9U, 9U, cost);
                e.store(9U, ctx, cycles, true);
            }
            e.bindLabel(cycles_done);
            e.storeImm(ctx, reason, static_cast<std::uint32_t>(why)); e.b(done);
        };
        exit(instruction_exit, NativeExitReason::instruction_limit, false, i);
        exit(cycle_exit, NativeExitReason::cycle_limit, false, i);
        exit(unsupported_exit, NativeExitReason::unsupported_instruction, true, i);
        exit(helper_exit, NativeExitReason::helper_declined, true, i);
        exit(mismatch_exit, NativeExitReason::branch_mismatch, true, i);
        if (op.stack && !op.push && (op.register_list & 0x8000U))
            exit(dynamic_return_exit, NativeExitReason::dynamic_return, false, i + 1U);
        else e.bindLabel(dynamic_return_exit);
        e.bindLabel(next);
    }
    const auto normal_checked_cycles = e.makeLabel();
    e.cbzX(22U, normal_checked_cycles);
    e.load(9U, 23U, ops.size() * sizeof(std::uint32_t));
    e.store(9U, ctx, cycles, true);
    e.bindLabel(normal_checked_cycles);
    e.bindLabel(done);
    e.load(19U, 31U, 0U, true); e.load(20U, 31U, 8U, true);
    e.load(21U, 31U, 16U, true); e.load(30U, 31U, 24U, true);
    e.load(22U, 31U, 32U, true); e.load(23U, 31U, 40U, true);
    e.emit(0x910103FFU); // add sp, sp, #64
    e.bindLabel(early); e.ret();
    if (!e.patchBranches(error)) return false;
    code.resize(e.words.size() * sizeof(std::uint32_t));
    std::memcpy(code.data(), e.words.data(), code.size());
    return true;
}

bool emitBlock(const NativeBlock& block, std::vector<std::uint8_t>& code, std::string& error) {
    std::vector<Op> ops;
    for (const auto& inst : block.ops) ops.push_back({inst, inst.pc + inst.size});
    return emitNative(ops, code, error);
}

bool emitTrace(const NativeTracePlan& plan, std::vector<std::uint8_t>& code, std::string& error) {
    std::vector<Op> ops;
    for (const auto& inst : plan.ops) {
        const bool stack = inst.decoded.kind == InstrKind::push || inst.decoded.kind == InstrKind::pop;
        auto native = inst.native;
        // Stack trace nodes have no scalar NativeInst; their source metadata
        // lives on the trace node itself.
        native.pc = inst.pc; native.size = inst.size;
        ops.push_back({native, inst.expected_next_pc, true, inst.expected_taken,
            stack, inst.decoded.kind == InstrKind::push, inst.decoded.register_list});
    }
    return emitNative(ops, code, error);
}

} // namespace aarch64detail
