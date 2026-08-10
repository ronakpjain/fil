#include "fil/cpu/cortex_m4.hpp"
#include "fil/elf/elf_loader.hpp"
#include "fil/mem/memory_bus.hpp"
#include "../test_support.hpp"

#include <bit>
#include <cstdint>
#include <filesystem>
#include <initializer_list>
#include <vector>

namespace {

constexpr std::uint32_t flash_base = 0x08000000U;
constexpr std::uint32_t ram_base = 0x20000000U;

class SharedMmio final : public fil::mem::MmioDevice {
public:
    fil::mem::MemoryResult<std::uint64_t> read(
        std::uint32_t, fil::mem::AccessSize, const fil::mem::AccessContext&
    ) override { return std::uint64_t{0}; }
    fil::mem::MemoryResult<std::uint64_t> write(
        std::uint32_t, fil::mem::AccessSize, std::uint64_t,
        const fil::mem::AccessContext&
    ) override {
        ++writes;
        return std::uint64_t{0};
    }
    std::string_view name() const noexcept override { return "shared-mmio"; }
    fil::mem::MmioDomain domain(
        std::uint32_t, fil::mem::AccessSize
    ) const noexcept override { return fil::mem::MmioDomain::shared; }

    unsigned int writes{0};
};

[[nodiscard]] std::vector<std::uint8_t> halfwords(
    const std::initializer_list<std::uint16_t> words
) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(words.size() * 2U);
    for (const std::uint16_t word : words) {
        bytes.push_back(static_cast<std::uint8_t>(word));
        bytes.push_back(static_cast<std::uint8_t>(word >> 8U));
    }
    return bytes;
}

[[nodiscard]] fil::mem::MemoryBus basicBus(const std::uint32_t flash_size = 256U) {
    fil::mem::MemoryBus bus;
    fil::test::check(bus.mapRom(flash_base, flash_size, "cpu-test-flash").hasValue(), "maps executable test flash");
    fil::test::check(bus.mapRam(ram_base, 256, "cpu-test-ram").hasValue(), "maps CPU test RAM");
    return bus;
}

void prepare(fil::cpu::CortexM4& cpu, const std::uint32_t pc = flash_base) {
    cpu.state() = fil::cpu::CpuState{};
    cpu.state().r[15] = pc;
    cpu.state().instruction_address = pc;
    cpu.state().msp = ram_base + 0xf0U;
    cpu.state().r[13] = cpu.state().msp;
}

void resetUsesElfVectorsAndSelectsActiveStack() {
    const auto fixture = std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
    const auto image = fil::elf::load(fixture);
    fil::test::check(image.hasValue(), "loads CPU reset ELF fixture");
    if (!image) return;

    auto bus = basicBus();
    fil::cpu::CortexM4 cpu(bus);
    fil::test::check(cpu.reset(image.value()), "accepts Thumb reset vector");
    fil::test::check(cpu.state().msp == image.value().initialMsp(), "reset initializes MSP from vector");
    fil::test::check(cpu.state().r[15] == (image.value().resetHandler() & ~1U), "reset clears stored PC Thumb bit");
    fil::test::check(cpu.state().thumb && (cpu.state().xpsr & fil::cpu::xpsr_t) != 0U, "reset establishes Thumb xPSR state");

    cpu.state().psp = 0x20000080U;
    cpu.state().control = 2U;
    fil::test::check(cpu.state().activeSp() == 0x20000080U, "thread mode CONTROL.SPSEL selects PSP");
    cpu.state().xpsr = (cpu.state().xpsr & ~fil::cpu::xpsr_ipsr_mask) | 3U;
    fil::test::check(cpu.state().activeSp() == image.value().initialMsp(), "handler mode always selects MSP");
}

void runsFromMaterializedElfResetVector() {
    const auto fixture = std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
    const auto image = fil::elf::load(fixture);
    fil::test::check(image.hasValue(), "loads executable CPU ELF fixture");
    if (!image) return;

    fil::mem::MemoryBus bus;
    fil::test::check(bus.mapRom(flash_base, 64, "fixture-flash").hasValue(), "maps ELF fixture flash");
    fil::test::check(bus.mapRam(ram_base, 64, "fixture-ram").hasValue(), "maps ELF fixture RAM");
    fil::test::check(bus.materialize(image.value()).hasValue(), "materializes CPU ELF fixture");
    fil::cpu::CortexM4 cpu(bus);
    fil::test::check(cpu.reset(image.value()), "resets CPU at materialized ELF handler");

    const auto result = cpu.run(8);
    fil::test::check(result.reason == fil::cpu::StopReason::breakpoint && result.instructions == 3U, "runs actual ELF reset handler to BKPT");
    fil::test::check(cpu.state().r[0] == ram_base && cpu.state().r[1] == ram_base + 4U, "ELF literal loads use aligned architectural PC");
}

void fetchesSixteenAndThirtyTwoBitInstructions() {
    auto bus = basicBus();
    const auto code = halfwords({0xbf00U, 0xf241U, 0x2034U});
    fil::test::check(bus.loadBytes(flash_base, code).hasValue(), "loads mixed-width test instructions");
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);

    const auto narrow = cpu.step();
    fil::test::check(narrow.reason == fil::cpu::StopReason::step_complete, "steps a 16-bit NOP");
    fil::test::check(narrow.diagnostic.instruction_address == flash_base && narrow.diagnostic.instruction_size == 2U, "reports precise 16-bit instruction address");
    fil::test::check(cpu.state().r[15] == flash_base + 2U, "16-bit fetch advances PC by two");

    const auto wide = cpu.step();
    fil::test::check(wide.reason == fil::cpu::StopReason::step_complete, "steps a 32-bit MOVW");
    fil::test::check(wide.diagnostic.raw == 0xf2412034U && wide.diagnostic.instruction_size == 4U, "reports both wide halfwords");
    fil::test::check(cpu.state().r[0] == 0x1234U && cpu.state().r[15] == flash_base + 6U, "executes wide instruction and advances by four");
}

void invalidatesDecodedInstructionsAfterExecutableWrites() {
    fil::mem::MemoryBus bus;
    fil::test::check(
        bus.mapRam(ram_base, 16U, "self-modifying-code", true).hasValue(),
        "maps executable RAM for decoded-cache invalidation"
    );
    const auto first_code = halfwords({0x2001U}); // movs r0, #1
    fil::test::check(
        bus.loadBytes(ram_base, first_code).hasValue(),
        "loads first self-modifying instruction"
    );
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu, ram_base);
    fil::test::check(
        cpu.step().reason == fil::cpu::StopReason::step_complete && cpu.state().r[0] == 1U,
        "executes and caches instruction from RAM"
    );

    fil::test::check(
        bus.write16(ram_base, 0x2002U).hasValue(),
        "rewrites cached executable instruction"
    );
    cpu.state().r[15] = ram_base;
    fil::test::check(
        cpu.step().reason == fil::cpu::StopReason::step_complete && cpu.state().r[0] == 2U,
        "executable write invalidates decoded instruction cache"
    );
}

void yieldsAndRestartsBeforeSharedMmio() {
    auto bus = basicBus();
    SharedMmio device;
    fil::test::check(bus.mapMmio(0x40000000U, 0x100U, device, "shared").hasValue()
                         && bus.loadBytes(flash_base, halfwords({0x6008U})).hasValue(),
                     "maps shared MMIO store fixture");
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    cpu.state().r[0] = 0x12345678U;
    cpu.state().r[1] = 0x40000000U;

    bus.setSharedMmioTrapping(true);
    const auto trapped = cpu.stepFast();
    fil::test::check(trapped.reason == fil::cpu::StopReason::synchronization_required
                         && trapped.instructions == 0U && trapped.cycles == 0U
                         && cpu.state().r[15] == flash_base && device.writes == 0U,
                     "CPU restores its pre-instruction state at shared MMIO");

    bus.setSharedMmioTrapping(false);
    const auto committed = cpu.stepFast();
    fil::test::check(committed.reason == fil::cpu::StopReason::step_complete
                         && cpu.state().r[15] == flash_base + 2U && device.writes == 1U,
                     "coordinator can restart and commit the trapped instruction once");
}

void reportsDecoderAndFetchFailuresWithoutLosingPc() {
    {
        auto bus = basicBus();
        const auto code = halfwords({0xde00U});
        fil::test::check(bus.loadBytes(flash_base, code).hasValue(), "loads reserved instruction");
        fil::cpu::CortexM4 cpu(bus);
        prepare(cpu);
        const auto result = cpu.step();
        fil::test::check(result.reason == fil::cpu::StopReason::undefined_instruction, "reserved encoding stops as undefined");
        fil::test::check(cpu.state().r[15] == flash_base, "undefined instruction leaves PC at faulting address");
        fil::test::check(result.diagnostic.raw == 0xde00U && result.diagnostic.message.find("pc=0x08000000") != std::string::npos, "undefined diagnostic includes raw encoding and PC");
    }
    {
        fil::mem::MemoryBus bus;
        fil::test::check(bus.mapRam(ram_base, 8, "non-executable").hasValue(), "maps non-executable fetch target");
        fil::cpu::CortexM4 cpu(bus);
        prepare(cpu, ram_base);
        const auto result = cpu.step();
        fil::test::check(result.reason == fil::cpu::StopReason::bus_fault, "execute-protected fetch returns bus fault");
        fil::test::check(result.diagnostic.bus_fault && result.diagnostic.bus_fault->context.type == fil::mem::AccessType::instruction_fetch, "fetch fault retains execute access metadata");
        fil::test::check(result.diagnostic.bus_fault && result.diagnostic.bus_fault->context.pc == ram_base, "fetch fault retains originating PC");
    }
    {
        fil::mem::MemoryBus bus;
        fil::test::check(bus.mapRom(flash_base, 2, "short-flash").hasValue(), "maps one-halfword executable region");
        const auto prefix = halfwords({0xf000U});
        fil::test::check(bus.loadBytes(flash_base, prefix).hasValue(), "loads lone wide prefix");
        fil::cpu::CortexM4 cpu(bus);
        prepare(cpu);
        const auto result = cpu.step();
        fil::test::check(result.reason == fil::cpu::StopReason::bus_fault, "missing second halfword returns fetch fault");
        fil::test::check(result.diagnostic.bus_fault && result.diagnostic.bus_fault->address == flash_base + 2U, "second-halfword fault reports its precise address");
        fil::test::check(cpu.state().r[15] == flash_base, "partial wide fetch leaves PC unchanged");
    }
}

void runsSyntheticStartupSliceToBreakpoint() {
    auto bus = basicBus();
    const auto code = halfwords({
        0x2005U,             // movs r0, #5
        0x2103U,             // movs r1, #3
        0x1842U,             // adds r2, r0, r1
        0x9200U,             // str r2, [sp]
        0x9b00U,             // ldr r3, [sp]
        0x2b08U,             // cmp r3, #8
        0xbf08U,             // it eq
        0x2409U,             // moveq r4, #9
        0xb403U,             // push {r0, r1}
        0xbc60U,             // pop {r5, r6}
        0xf000U, 0xf801U,    // bl subroutine (+2)
        0xbe00U,             // bkpt #0
        0x3701U,             // subroutine: adds r7, #1
        0x4770U,             // bx lr
    });
    fil::test::check(bus.loadBytes(flash_base, code).hasValue(), "loads synthetic startup instruction slice");
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    const auto initial_sp = cpu.state().activeSp();

    const auto result = cpu.run(32);
    fil::test::check(result.reason == fil::cpu::StopReason::breakpoint, "startup slice stops at BKPT");
    fil::test::check(result.instructions == 14U, "run reports exact executed instruction count");
    fil::test::check(cpu.state().r[2] == 8U && cpu.state().r[3] == 8U, "startup ALU/store/load round-trip is exact");
    fil::test::check(cpu.state().r[4] == 9U, "IT-conditioned startup instruction executes");
    fil::test::check(cpu.state().r[5] == 5U && cpu.state().r[6] == 3U, "PUSH/POP preserve ascending register order");
    fil::test::check(cpu.state().r[7] == 1U, "BL/BX executes and returns from subroutine");
    fil::test::check(cpu.state().activeSp() == initial_sp, "balanced PUSH/POP restores active SP");
    fil::test::check(cpu.state().it_state == 0U, "IT state advances after its controlled instruction");
}

void marksCallsAndReturnsAsLoopProofBarriers() {
    auto bus = basicBus();
    const auto code = halfwords({
        0xf000U, 0xf801U, // bl subroutine (+2)
        0xbe00U,          // bkpt #0
        0x3001U,          // subroutine: adds r0, #1
        0x4770U,          // bx lr
    });
    fil::test::check(bus.loadBytes(flash_base, code).hasValue(),
                     "loads call/return loop-proof fixture");
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);

    const auto call = cpu.stepFast();
    const auto body = cpu.stepFast();
    const auto return_step = cpu.stepFast();
    fil::test::check(call.reason == fil::cpu::StopReason::step_complete
                         && call.suppress_loop_observation,
                     "suppresses loop observation after a direct call");
    fil::test::check(body.reason == fil::cpu::StopReason::step_complete
                         && !body.suppress_loop_observation,
                     "keeps ordinary instructions eligible for loop observation");
    fil::test::check(return_step.reason == fil::cpu::StopReason::step_complete
                         && return_step.suppress_loop_observation
                         && cpu.state().r[15] == flash_base + 4U,
                     "suppresses loop observation after a standard return");
}

void executesLoadStoreWidthsAndPcPop() {
    auto bus = basicBus();
    const auto code = halfwords({
        0x70caU, // strb r2, [r1, #3]
        0x78cbU, // ldrb r3, [r1, #3]
        0x808aU, // strh r2, [r1, #4]
        0x888cU, // ldrh r4, [r1, #4]
        0xbd00U, // pop {pc}
    });
    fil::test::check(bus.loadBytes(flash_base, code).hasValue(), "loads width and PC-load instruction slice");
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    cpu.state().r[1] = ram_base;
    cpu.state().r[2] = 0x1234abcdU;
    const auto return_word = halfwords({0x0021U, 0x0800U});
    fil::test::check(bus.loadBytes(cpu.state().activeSp(), return_word).hasValue(), "loads odd POP return target");

    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes STRB");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes LDRB");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes STRH");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes LDRH");
    fil::test::check(cpu.state().r[3] == 0xcdU && cpu.state().r[4] == 0xabcdU, "byte and halfword loads zero-extend correctly");
    const auto pop_pc = cpu.stepFast();
    fil::test::check(pop_pc.reason == fil::cpu::StopReason::step_complete,
                     "executes POP PC with Thumb target");
    fil::test::check(cpu.state().r[15] == 0x08000020U,
                     "POP PC validates and clears target Thumb bit");
    fil::test::check(pop_pc.suppress_loop_observation,
                     "suppresses loop observation after a stack return");
}

void boundedRunAndDataFaultsAreStructured() {
    {
        auto bus = basicBus();
        const auto code = halfwords({0xbf00U, 0xbf00U, 0xbf00U});
        fil::test::check(bus.loadBytes(flash_base, code).hasValue(), "loads NOP budget fixture");
        fil::cpu::CortexM4 cpu(bus);
        prepare(cpu);
        const auto result = cpu.run(2);
        fil::test::check(result.reason == fil::cpu::StopReason::instruction_budget && result.instructions == 2U, "bounded run stops exactly at instruction budget");
        fil::test::check(cpu.state().r[15] == flash_base + 4U, "budget stop preserves next PC");
    }
    {
        auto bus = basicBus();
        const auto code = halfwords({0x6008U}); // str r0, [r1]
        fil::test::check(bus.loadBytes(flash_base, code).hasValue(), "loads faulting store");
        fil::cpu::CortexM4 cpu(bus);
        prepare(cpu);
        cpu.state().r[1] = flash_base;
        const auto result = cpu.step();
        fil::test::check(result.reason == fil::cpu::StopReason::bus_fault, "data permission failure stops CPU");
        fil::test::check(result.diagnostic.bus_fault && result.diagnostic.bus_fault->context.type == fil::mem::AccessType::data_write, "data fault retains write access type");
        fil::test::check(result.diagnostic.bus_fault && result.diagnostic.bus_fault->context.pc == flash_base, "data fault retains precise instruction PC");
    }
}

void itAlwaysInstallsConditionBeforeConditionalVfpContextTransfer() {
    auto bus = basicBus();
    const auto code = halfwords({
        0xf01eU, 0x0f10U, // tst.w lr, #16
        0xbf08U,          // it eq
        0xecb0U, 0x8a10U, // vldmiaeq r0!, {s16-s31}
        0xbf00U,
    });
    fil::test::check(bus.loadBytes(flash_base, code).hasValue(), "loads real PendSV conditional VFP slice");
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    cpu.state().r[0] = ram_base + 0x40U;
    cpu.state().r[14] = 0xfffffffdU;

    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes PendSV TST.W");
    fil::test::check((cpu.state().xpsr & fil::cpu::xpsr_z) == 0U, "TST sees basic EXC_RETURN bit set");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "IT executes even when its EQ condition is false");
    fil::test::check(cpu.state().it_state != 0U, "IT installs its condition state unconditionally");
    const std::uint32_t original_psp = cpu.state().r[0];
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "false VLDMEQ slot is skipped without a fault");
    fil::test::check(cpu.state().r[0] == original_psp, "skipped VLDMEQ does not advance task stack pointer");
    fil::test::check(cpu.state().it_state == 0U, "false conditional VFP slot still advances IT state");
}

void executesRealWideAluMultiplyAndBitfieldEncodings() {
    auto bus = basicBus();
    const auto code = halfwords({
        0xf443U, 0x0370U, // orr.w r3, r3, #0x00f00000
        0xebb6U, 0x0fa5U, // cmp.w r6, r5, asr #2
        0xea4fU, 0x0c93U, // mov.w ip, r3, lsr #2
        0xfa03U, 0xf000U, // lsl.w r0, r3, r0
        0xfbb5U, 0xf2f6U, // udiv r2, r5, r6
        0xfb06U, 0x5212U, // mls r2, r6, r2, r5
        0xfb02U, 0xf303U, // mul.w r3, r2, r3
        0xfab2U, 0xf282U, // clz r2, r2
        0xf3c3U, 0x1303U, // ubfx r3, r3, #4, #4
        0xfb01U, 0x3002U, // mla r0, r1, r2, r3
        0xfba1U, 0x1303U, // umull r1, r3, r1, r3
        0xf602U, 0x420cU, // addw r2, r2, #0xc0c
    });
    fil::test::check(bus.loadBytes(flash_base, code).hasValue(), "loads real wide ALU and multiply encodings");
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);

    cpu.state().r[3] = 0x123U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes ORR.W modified immediate");
    fil::test::check(cpu.state().r[3] == 0x00f00123U, "Thumb modified immediate expands exactly");

    cpu.state().r[6] = 10U;
    cpu.state().r[5] = 8U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes CMP.W shifted register");
    fil::test::check((cpu.state().xpsr & fil::cpu::xpsr_c) != 0U && (cpu.state().xpsr & fil::cpu::xpsr_z) == 0U, "CMP.W uses shifted operand and updates flags");

    cpu.state().r[3] = 0x40U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes MOV.W shifted register");
    fil::test::check(cpu.state().r[12] == 0x10U, "MOV.W applies immediate LSR");

    cpu.state().r[3] = 3U;
    cpu.state().r[0] = 4U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes register-controlled LSL.W");
    fil::test::check(cpu.state().r[0] == 48U, "LSL.W takes low byte of shift register");

    cpu.state().r[5] = 101U;
    cpu.state().r[6] = 10U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes UDIV");
    fil::test::check(cpu.state().r[2] == 10U, "UDIV computes unsigned quotient");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes MLS");
    fil::test::check(cpu.state().r[2] == 1U, "MLS computes remainder idiom");

    cpu.state().r[2] = 7U;
    cpu.state().r[3] = 9U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes MUL.W");
    fil::test::check(cpu.state().r[3] == 63U, "MUL.W keeps low product");

    cpu.state().r[2] = 0x1000U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes CLZ");
    fil::test::check(cpu.state().r[2] == 19U, "CLZ counts all leading zeroes");

    cpu.state().r[3] = 0x0ab0U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes UBFX");
    fil::test::check(cpu.state().r[3] == 0x0bU, "UBFX extracts requested bit field");

    cpu.state().r[1] = 7U;
    cpu.state().r[2] = 6U;
    cpu.state().r[3] = 5U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes MLA");
    fil::test::check(cpu.state().r[0] == 47U, "MLA multiplies then accumulates with wrap semantics");

    cpu.state().r[1] = 0xffffffffU;
    cpu.state().r[3] = 2U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes UMULL");
    fil::test::check(cpu.state().r[1] == 0xfffffffeU && cpu.state().r[3] == 1U, "UMULL writes low and high halves");

    cpu.state().r[2] = 0x1000U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes ADDW plain immediate");
    fil::test::check(cpu.state().r[2] == 0x1c0cU, "ADDW reconstructs all twelve immediate bits");
}

void executesExtendDoublewordAndVfpContextTransfers() {
    auto bus = basicBus();
    const auto code = halfwords({
        0xb2f6U,             // uxtb r6, r6
        0xb264U,             // sxtb r4, r4
        0xfa1fU, 0xfc8cU,    // uxth.w ip, ip
        0xe96dU, 0xce04U,    // strd ip, lr, [sp, #-16]!
        0xe9ddU, 0x2302U,    // ldrd r2, r3, [sp, #8]
        0xed20U, 0x8a10U,    // vstmdb r0!, {s16-s31}
        0xecb0U, 0x8a10U,    // vldmia r0!, {s16-s31}
    });
    fil::test::check(bus.loadBytes(flash_base, code).hasValue(), "loads extend, doubleword, and VFP context encodings");
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    cpu.state().r[6] = 0x123456feU;
    cpu.state().r[4] = 0x80U;
    cpu.state().r[12] = 0x12345678U;

    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes UXTB");
    fil::test::check(cpu.state().r[6] == 0xfeU, "UXTB zero extends byte");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes SXTB");
    fil::test::check(cpu.state().r[4] == 0xffffff80U, "SXTB sign extends byte");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes UXTH.W");
    fil::test::check(cpu.state().r[12] == 0x5678U, "UXTH.W zero extends halfword");

    const std::uint32_t initial_sp = cpu.state().activeSp();
    cpu.state().r[14] = 0x89abcdefU;
    fil::test::check(bus.write32(initial_sp - 8U, 0x11223344U).hasValue(), "seeds first LDRD word");
    fil::test::check(bus.write32(initial_sp - 4U, 0xaabbccddU).hasValue(), "seeds second LDRD word");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes pre-decrement STRD with LR as Rt2");
    fil::test::check(cpu.state().activeSp() == initial_sp - 16U, "STRD writeback decrements SP once");
    const auto stored_ip = bus.read32(initial_sp - 16U);
    const auto stored_lr = bus.read32(initial_sp - 12U);
    fil::test::check(stored_ip && stored_ip.value() == 0x5678U, "STRD stores first register");
    fil::test::check(stored_lr && stored_lr.value() == 0x89abcdefU, "STRD accepts and stores LR as second register");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes LDRD with positive offset");
    fil::test::check(cpu.state().r[2] == 0x11223344U && cpu.state().r[3] == 0xaabbccddU, "LDRD loads both consecutive words");

    cpu.state().r[0] = ram_base + 0xc0U;
    for (std::size_t index = 16U; index < 32U; ++index) {
        cpu.state().s[index] = static_cast<float>(index) + 0.25F;
    }
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes FreeRTOS VSTMDB context save");
    fil::test::check(cpu.state().r[0] == ram_base + 0x80U, "VSTMDB writes back decremented base");
    for (std::size_t index = 16U; index < 32U; ++index) cpu.state().s[index] = 0.0F;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes FreeRTOS VLDMIA context restore");
    fil::test::check(cpu.state().r[0] == ram_base + 0xc0U, "VLDMIA writes back incremented base");
    fil::test::check(cpu.state().s[16] == 16.25F && cpu.state().s[31] == 31.25F, "VFP context transfer preserves s16-s31 bits");
}

void executesSystemInstructionsAndRaisesConsumableMarkers() {
    auto bus = basicBus();
    const auto code = halfwords({
        0xf3efU, 0x8305U, // mrs r3, IPSR
        0xf383U, 0x8811U, // msr BASEPRI, r3
        0xf380U, 0x8809U, // msr PSP, r0
        0xf380U, 0x8814U, // msr CONTROL, r0
        0xb672U,          // cpsid i
        0xb662U,          // cpsie i
        0xf3bfU, 0x8f4fU, // dsb sy
        0xf3bfU, 0x8f5fU, // dmb sy
        0xf3bfU, 0x8f6fU, // isb sy
        0xdf2aU,          // svc #42
        0x4770U,          // bx lr
        0xbd00U,          // pop {pc}
    });
    fil::test::check(bus.loadBytes(flash_base, code).hasValue(), "loads real system and exception-return encodings");
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);
    cpu.state().xpsr = (cpu.state().xpsr & ~fil::cpu::xpsr_ipsr_mask) | 11U;

    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes MRS IPSR");
    fil::test::check(cpu.state().r[3] == 11U, "MRS reads IPSR exception number");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes MSR BASEPRI");
    fil::test::check(cpu.state().basepri == 11U, "MSR writes BASEPRI low byte");

    cpu.state().xpsr &= ~fil::cpu::xpsr_ipsr_mask;
    cpu.state().r[0] = ram_base + 0x80U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes MSR PSP");
    fil::test::check(cpu.state().psp == ram_base + 0x80U, "MSR writes aligned PSP");
    cpu.state().r[0] = 3U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes MSR CONTROL");
    fil::test::check(cpu.state().control == 3U && cpu.state().activeSp() == ram_base + 0x80U, "CONTROL selects PSP in thread mode");

    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete && cpu.state().primask == 1U, "CPSID i masks interrupts");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete && cpu.state().primask == 0U, "CPSIE i unmasks interrupts");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "DSB is a deterministic CPU no-op");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "DMB is a deterministic CPU no-op");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "ISB is a deterministic CPU no-op");

    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes SVC");
    fil::test::check(cpu.state().pending_exception && *cpu.state().pending_exception == 11U, "SVC requests architectural exception 11");

    cpu.state().r[14] = 0xfffffffdU;
    const std::uint32_t pc_after_bx = cpu.state().r[15] + 2U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes BX EXC_RETURN");
    fil::test::check(cpu.state().pending_exc_return && *cpu.state().pending_exc_return == 0xfffffffdU, "BX exposes consumable EXC_RETURN marker");
    fil::test::check(cpu.state().r[15] == pc_after_bx, "BX EXC_RETURN never fetches from 0xfffffffX");

    cpu.state().pending_exc_return.reset();
    fil::test::check(bus.write32(cpu.state().activeSp(), 0xfffffff9U).hasValue(), "seeds POP EXC_RETURN");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes POP EXC_RETURN");
    fil::test::check(cpu.state().pending_exc_return && *cpu.state().pending_exc_return == 0xfffffff9U, "POP exposes consumable EXC_RETURN marker");
}

void executesRealScalarVfpPipeline() {
    auto bus = basicBus();
    const auto code = halfwords({
        0xee07U, 0x6a90U, // vmov s15, r6
        0xeeb8U, 0x0a67U, // vcvt.f32.u32 s0, s15
        0xee10U, 0x3a10U, // vmov r3, s0
        0xeef7U, 0x7a00U, // vmov.f32 s15, #1.0
        0xed82U, 0x7a00U, // vstr s14, [r2]
        0xed92U, 0x7a00U, // vldr s14, [r2]
        0xee37U, 0x7a20U, // vadd.f32 s14, s14, s1
        0xee77U, 0x7a67U, // vsub.f32 s15, s14, s15
        0xee27U, 0x7a87U, // vmul.f32 s14, s15, s14
        0xeec6U, 0x7a87U, // vdiv.f32 s15, s13, s14
        0xeeb4U, 0x7ac0U, // vcmpe.f32 s14, s0
        0xeef1U, 0xfa10U, // vmrs APSR_nzcv, fpscr
        0xeef8U, 0x7ae7U, // vcvt.f32.s32 s15, s15
        0xeefdU, 0x7ae7U, // vcvt.s32.f32 s15, s15
        0xee17U, 0x3a90U, // vmov r3, s15
    });
    fil::test::check(bus.loadBytes(flash_base, code).hasValue(), "loads real scalar VFP pipeline");
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);

    cpu.state().r[6] = 42U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes core-to-S VMOV");
    fil::test::check(std::bit_cast<std::uint32_t>(cpu.state().s[15]) == 42U, "core-to-S VMOV is bit exact");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes VCVT.F32.U32");
    fil::test::check(cpu.state().s[0] == 42.0F, "unsigned integer converts to float32");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes S-to-core VMOV");
    fil::test::check(cpu.state().r[3] == std::bit_cast<std::uint32_t>(42.0F), "S-to-core VMOV preserves float bits");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes VMOV.F32 immediate");
    fil::test::check(cpu.state().s[15] == 1.0F, "VFP immediate expands to 1.0 exactly");

    cpu.state().r[2] = ram_base;
    cpu.state().s[14] = 3.25F;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes scalar VSTR");
    cpu.state().s[14] = 0.0F;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes scalar VLDR");
    fil::test::check(cpu.state().s[14] == 3.25F, "scalar VSTR/VLDR round-trip preserves bits");

    cpu.state().s[1] = 0.75F;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes VADD.F32");
    fil::test::check(cpu.state().s[14] == 4.0F, "VADD uses split S-register fields");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes VSUB.F32");
    fil::test::check(cpu.state().s[15] == 3.0F, "VSUB computes ordered operands");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes VMUL.F32");
    fil::test::check(cpu.state().s[14] == 12.0F, "VMUL computes float32 product");
    cpu.state().s[13] = 24.0F;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes VDIV.F32");
    fil::test::check(cpu.state().s[15] == 2.0F, "VDIV computes float32 quotient");

    cpu.state().s[14] = 1.0F;
    cpu.state().s[0] = 2.0F;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes VCMPE.F32");
    fil::test::check((cpu.state().fpscr & fil::cpu::xpsr_n) != 0U, "VCMP writes FPSCR less-than flags");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes VMRS APSR_nzcv,FPSCR");
    fil::test::check((cpu.state().xpsr & fil::cpu::xpsr_n) != 0U, "VMRS transfers FPSCR comparison flags");

    cpu.state().s[15] = std::bit_cast<float>(0xfffffff9U);
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes VCVT.F32.S32");
    fil::test::check(cpu.state().s[15] == -7.0F, "signed integer converts to float32");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes VCVT.S32.F32");
    fil::test::check(std::bit_cast<std::uint32_t>(cpu.state().s[15]) == 0xfffffff9U, "float32 converts toward zero to signed integer bits");
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "moves converted integer bits to core register");
    fil::test::check(cpu.state().r[3] == 0xfffffff9U, "converted integer reaches core register unchanged");
}

void executesRemainingRealBoardIntegerEncodings() {
    auto bus = basicBus();
    const auto code = halfwords({
        0xba5bU,             // rev16 r3, r3
        0xfb93U, 0xf1f2U,    // sdiv r1, r3, r2
        0xfb82U, 0x1203U,    // smull r1, r2, r2, r3
        0xfa85U, 0xf547U,    // uadd8 r5, r5, r7
        0xfaa3U, 0xf587U,    // sel r5, r3, r7
    });
    fil::test::check(bus.loadBytes(flash_base, code).hasValue(), "loads residual production-board integer encodings");
    fil::cpu::CortexM4 cpu(bus);
    prepare(cpu);

    cpu.state().r[3] = 0x11223344U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes REV16");
    fil::test::check(cpu.state().r[3] == 0x22114433U, "REV16 swaps bytes within both halfwords");

    cpu.state().r[3] = std::bit_cast<std::uint32_t>(-10);
    cpu.state().r[2] = 3U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes SDIV");
    fil::test::check(cpu.state().r[1] == std::bit_cast<std::uint32_t>(-3), "SDIV rounds signed quotient toward zero");

    cpu.state().r[2] = std::bit_cast<std::uint32_t>(-2);
    cpu.state().r[3] = 3U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes SMULL");
    fil::test::check(cpu.state().r[1] == 0xfffffffaU && cpu.state().r[2] == 0xffffffffU, "SMULL writes signed low and high halves");

    cpu.state().r[5] = 0xff01fe80U;
    cpu.state().r[7] = 0x01020390U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes UADD8");
    fil::test::check(cpu.state().r[5] == 0x00030110U, "UADD8 wraps each byte independently");
    fil::test::check((cpu.state().xpsr & fil::cpu::xpsr_ge_mask) == 0x000b0000U, "UADD8 records per-byte carry in APSR.GE");

    cpu.state().r[3] = 0xa1b2c3d4U;
    cpu.state().r[7] = 0x11223344U;
    fil::test::check(cpu.step().reason == fil::cpu::StopReason::step_complete, "executes SEL");
    fil::test::check(cpu.state().r[5] == 0xa122c3d4U, "SEL chooses each byte from APSR.GE lane");
}

} // namespace

void runCpuStepTests() {
    resetUsesElfVectorsAndSelectsActiveStack();
    runsFromMaterializedElfResetVector();
    fetchesSixteenAndThirtyTwoBitInstructions();
    invalidatesDecodedInstructionsAfterExecutableWrites();
    yieldsAndRestartsBeforeSharedMmio();
    reportsDecoderAndFetchFailuresWithoutLosingPc();
    runsSyntheticStartupSliceToBreakpoint();
    marksCallsAndReturnsAsLoopProofBarriers();
    executesLoadStoreWidthsAndPcPop();
    boundedRunAndDataFaultsAreStructured();
    itAlwaysInstallsConditionBeforeConditionalVfpContextTransfer();
    executesRealWideAluMultiplyAndBitfieldEncodings();
    executesExtendDoublewordAndVfpContextTransfers();
    executesSystemInstructionsAndRaisesConsumableMarkers();
    executesRealScalarVfpPipeline();
    executesRemainingRealBoardIntegerEncodings();
}
