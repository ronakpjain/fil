#include "fil/cpu/llvm_jit.hpp"

#include "fil/cpu/cortex_m4.hpp"

#include <llvm/ExecutionEngine/Orc/LLJIT.h>
#include <llvm/ExecutionEngine/Orc/ThreadSafeModule.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/PassManager.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Passes/OptimizationLevel.h>
#include <llvm/Passes/PassBuilder.h>
#include <llvm/Support/Error.h>
#include <llvm/Config/llvm-config.h>
#include <type_traits>
#include <llvm/Support/TargetSelect.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <utility>

namespace fil::cpu {
namespace {

using llvm::BasicBlock;
using llvm::ConstantInt;
using llvm::Function;
using llvm::IRBuilder;
using llvm::LLVMContext;
using llvm::Module;
using llvm::Type;
using llvm::Value;

static_assert(std::is_standard_layout_v<CpuState>, "Native JIT requires offsetof-compatible CPU state");

constexpr std::uint32_t flag_mask = xpsr_n | xpsr_z | xpsr_c | xpsr_v;
using NativeFunction = void (*)(CpuState*, std::size_t);

std::atomic<std::uint64_t> next_module_id{0};

Value* c32(LLVMContext& context, const std::uint32_t value) {
    return ConstantInt::get(Type::getInt32Ty(context), value);
}
Value* c64(LLVMContext& context, const std::uint64_t value) {
    return ConstantInt::get(Type::getInt64Ty(context), value);
}

bool verifyModule(Module& module, std::string& error) {
    std::string diagnostics;
    llvm::raw_string_ostream output(diagnostics);
    if (!llvm::verifyModule(module, &output)) return true;
    output.flush();
    error = std::move(diagnostics);
    return false;
}

bool onlyImmediateShape(const DecodedInstruction& d) noexcept {
    return d.form == OperandForm::immediate && d.rm == 0U && d.ra == 0U
        && d.shift_amount == 0U && d.shift_type == ShiftType::lsl
        && !d.writeback && d.register_list == 0U && !d.fp_double;
}

Value* stateField(IRBuilder<>& b, LLVMContext& c, Value* state, const std::size_t offset) {
    auto* byte = Type::getInt8Ty(c);
    auto* ptr = b.CreateGEP(byte, state, c64(c, offset));
    return ptr;
}

Value* loadField(IRBuilder<>& b, LLVMContext& c, Value* state, const std::size_t offset) {
    return b.CreateLoad(Type::getInt32Ty(c), stateField(b, c, state, offset));
}
void storeField(IRBuilder<>& b, LLVMContext& c, Value* state, const std::size_t offset, Value* value) {
    b.CreateStore(value, stateField(b, c, state, offset));
}
Value* loadReg(IRBuilder<>& b, LLVMContext& c, Value* state, std::uint8_t reg) {
    return loadField(b, c, state, offsetof(CpuState, r) + sizeof(std::uint32_t) * reg);
}
void storeReg(IRBuilder<>& b, LLVMContext& c, Value* state, std::uint8_t reg, Value* value) {
    storeField(b, c, state, offsetof(CpuState, r) + sizeof(std::uint32_t) * reg, value);
}

void writeNzcv(IRBuilder<>& b, LLVMContext& c, Value* state, Value* result,
               Value* carry, Value* overflow) {
    Value* old = loadField(b, c, state, offsetof(CpuState, xpsr));
    Value* retained = b.CreateAnd(old, c32(c, ~flag_mask));
    Value* n = b.CreateAnd(result, c32(c, xpsr_n));
    Value* z = b.CreateZExt(b.CreateICmpEQ(result, c32(c, 0)), Type::getInt32Ty(c));
    z = b.CreateShl(z, c32(c, 30));
    Value* cb = b.CreateShl(b.CreateZExt(carry, Type::getInt32Ty(c)), c32(c, 29));
    Value* vb = b.CreateShl(b.CreateZExt(overflow, Type::getInt32Ty(c)), c32(c, 28));
    storeField(b, c, state, offsetof(CpuState, xpsr), b.CreateOr(retained, b.CreateOr(n, b.CreateOr(z, b.CreateOr(cb, vb)))));
}

void writeNzc(IRBuilder<>& b, LLVMContext& c, Value* state, Value* result, Value* carry) {
    Value* old = loadField(b, c, state, offsetof(CpuState, xpsr));
    Value* retained = b.CreateAnd(old, c32(c, ~(xpsr_n | xpsr_z | xpsr_c)));
    Value* n = b.CreateAnd(result, c32(c, xpsr_n));
    Value* z = b.CreateShl(b.CreateZExt(b.CreateICmpEQ(result, c32(c, 0)), Type::getInt32Ty(c)), c32(c, 30));
    Value* cb = b.CreateShl(b.CreateZExt(carry, Type::getInt32Ty(c)), c32(c, 29));
    storeField(b, c, state, offsetof(CpuState, xpsr), b.CreateOr(retained, b.CreateOr(n, b.CreateOr(z, cb))));
}

void emitAddFlags(IRBuilder<>& b, LLVMContext& c, Value* state, Value* left, Value* right,
                  bool subtract) {
    Value* y = subtract ? b.CreateNot(right) : right;
    Value* sum64 = b.CreateAdd(b.CreateZExt(left, Type::getInt64Ty(c)),
                               b.CreateZExt(y, Type::getInt64Ty(c)));
    if (subtract) sum64 = b.CreateAdd(sum64, c64(c, 1));
    Value* result = b.CreateTrunc(sum64, Type::getInt32Ty(c));
    Value* carry = b.CreateICmpNE(b.CreateLShr(sum64, c64(c, 32)), c64(c, 0));
    Value* xor1 = b.CreateXor(left, y);
    Value* xor2 = b.CreateXor(left, result);
    Value* ovbits = b.CreateAnd(b.CreateAnd(b.CreateNot(xor1), xor2), c32(c, 0x80000000U));
    if (subtract) {
        // subtraction is x + ~y + 1; its signed overflow formula uses the original y.
        ovbits = b.CreateAnd(b.CreateXor(left, right), b.CreateXor(left, result));
        ovbits = b.CreateAnd(ovbits, c32(c, 0x80000000U));
    }
    Value* overflow = b.CreateICmpNE(ovbits, c32(c, 0));
    writeNzcv(b, c, state, result, carry, overflow);
}

bool shapeSupported(const NativeJitInstruction& i) noexcept {
    const auto& d = i.decoded;
    if ((i.pc & 1U) != 0U || (i.size != 2U && i.size != 4U)) return false;
    if (d.condition != Condition::al || d.is_32bit != (i.size == 4U)) return false;
    switch (d.kind) {
    case InstrKind::nop:
        return d.form == OperandForm::none && !d.set_flags && d.rd == 0U && d.rn == 0U
            && d.rm == 0U && d.imm == 0U && d.branch_offset == 0;
    case InstrKind::mov:
        return onlyImmediateShape(d) && d.rd <= 12U;
    case InstrKind::movw:
        return onlyImmediateShape(d) && d.rd <= 12U && d.rn == 0U && d.imm <= 0xffffU
            && !d.set_flags && !d.immediate_carry_valid && !d.immediate_carry;
    case InstrKind::movt:
        return onlyImmediateShape(d) && d.rd <= 12U && d.rn == 0U && d.imm <= 0xffffU
            && !d.set_flags && !d.immediate_carry_valid && !d.immediate_carry;
    case InstrKind::add:
    case InstrKind::sub:
        return onlyImmediateShape(d) && d.rd <= 12U && d.rn <= 12U
            && !d.immediate_carry_valid && !d.immediate_carry;
    case InstrKind::cmp:
    case InstrKind::cmn:
        return onlyImmediateShape(d) && d.rn <= 12U && d.rd <= 15U && d.set_flags
            && !d.immediate_carry_valid && !d.immediate_carry;
    case InstrKind::b:
    case InstrKind::bl:
        return (d.form == OperandForm::none || d.form == OperandForm::immediate)
            && !d.set_flags && d.rd == 0U && d.rn == 0U
            && d.rm == 0U && d.imm == 0U && !d.immediate_carry_valid && !d.immediate_carry;
    default:
        return false;
    }
}

} // namespace

struct LlvmJit::Impl {
    std::unique_ptr<llvm::orc::LLJIT> jit;
    std::mutex mutex;
};

struct NativeJitKernel::Impl {
    std::shared_ptr<LlvmJit::Impl> runtime;
    llvm::orc::ResourceTrackerSP tracker;
    NativeFunction function{nullptr};
};

NativeJitKernel::NativeJitKernel(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
NativeJitKernel::~NativeJitKernel() {
    if (impl_ && impl_->tracker) {
        std::lock_guard lock(impl_->runtime->mutex);
        llvm::consumeError(impl_->tracker->remove());
    }
}
NativeJitKernel::Function NativeJitKernel::entryPoint() const noexcept {
    return impl_ ? impl_->function : nullptr;
}

void NativeJitKernel::execute(CpuState& state, const std::size_t max_instructions) const noexcept {
    if (max_instructions == 0U || !impl_ || impl_->function == nullptr) return;
    // AL instructions inside IT are not unconditional architecturally. Keep
    // scheduler/control context and exceptional state entirely untouched.
    if (state.it_state != 0U || state.halted || !state.thumb || (state.xpsr & xpsr_t) == 0U
        || state.pending_exception.has_value() || state.pending_exc_return.has_value()) return;
    impl_->function(&state, max_instructions);
}

LlvmJit::LlvmJit(std::shared_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
LlvmJit::~LlvmJit() = default;

std::unique_ptr<LlvmJit> LlvmJit::create(std::string& error) {
    error.clear();
    if (sizeof(std::size_t) != 8U) {
        error = "LLVM native JIT requires a 64-bit size_t ABI";
        return nullptr;
    }
    static std::once_flag initialized;
    static std::string initialization_error;
    std::call_once(initialized, [] {
        if (llvm::InitializeNativeTarget()) initialization_error = "LLVM native target initialization failed";
        else if (llvm::InitializeNativeTargetAsmPrinter()) initialization_error = "LLVM native asm-printer initialization failed";
        else if (llvm::InitializeNativeTargetAsmParser()) initialization_error = "LLVM native asm-parser initialization failed";
    });
    if (!initialization_error.empty()) { error = initialization_error; return nullptr; }
    auto expected = llvm::orc::LLJITBuilder().create();
    if (!expected) { error = llvm::toString(expected.takeError()); return nullptr; }
    auto impl = std::make_shared<Impl>();
    impl->jit = std::move(*expected);
    return std::unique_ptr<LlvmJit>(new LlvmJit(std::move(impl)));
}

bool LlvmJit::supports(const NativeJitInstruction& instruction) noexcept {
    return shapeSupported(instruction);
}

std::shared_ptr<const NativeJitKernel> LlvmJit::compile(
    const std::span<const NativeJitInstruction> instructions, std::string& error) {
    error.clear();
    if (sizeof(std::size_t) != 8U) {
        error = "LLVM native JIT requires a 64-bit size_t ABI";
        return {};
    }
    if (instructions.empty()) { error = "cannot compile an empty native block"; return {}; }
    bool terminated = false;
    for (std::size_t i = 0; i < instructions.size(); ++i) {
        if (!supports(instructions[i])) {
            error = "unsupported native JIT instruction at index " + std::to_string(i);
            return {};
        }
        if (i != 0U && instructions[i].pc != instructions[i - 1U].pc + instructions[i - 1U].size) {
            error = "native JIT instruction addresses are not contiguous";
            return {};
        }
        const auto kind = instructions[i].decoded.kind;
        if (terminated) { error = "instruction follows a native branch terminator"; return {}; }
        terminated = kind == InstrKind::b || kind == InstrKind::bl;
        if (terminated && i + 1U != instructions.size()) {
            error = "native branch terminator must be the final instruction";
            return {};
        }
    }

    auto context = std::make_unique<LLVMContext>();
    const auto module_id = next_module_id.fetch_add(1);
    auto module = std::make_unique<Module>("fil.native." + std::to_string(module_id), *context);
#if LLVM_VERSION_MAJOR >= 21
    module->setTargetTriple(impl_->jit->getTargetTriple());
#else
    module->setTargetTriple(impl_->jit->getTargetTriple().str());
#endif
    module->setDataLayout(impl_->jit->getDataLayout());
    auto* voidTy = Type::getVoidTy(*context);
    auto* ptrTy = llvm::PointerType::getUnqual(*context);
    auto* fnTy = llvm::FunctionType::get(voidTy, {ptrTy, Type::getInt64Ty(*context)}, false);
    auto* fn = Function::Create(fnTy, Function::ExternalLinkage,
                                "fil_kernel_" + std::to_string(module_id), *module);
    auto args = fn->arg_begin();
    Value* state = &*args++;
    Value* maximum = &*args;
    state->setName("state"); maximum->setName("maximum");
    IRBuilder<> b(*context);
    auto* entry = BasicBlock::Create(*context, "entry", fn);
    b.SetInsertPoint(entry);
    for (std::size_t i = 0; i < instructions.size(); ++i) {
        auto* run = BasicBlock::Create(*context, "op" + std::to_string(i), fn);
        auto* next = BasicBlock::Create(*context, "next" + std::to_string(i), fn);
        b.CreateCondBr(b.CreateICmpUGT(maximum, c64(*context, i)), run, next);
        b.SetInsertPoint(run);
        const auto& item = instructions[i];
        const auto& d = item.decoded;
        storeField(b, *context, state, offsetof(CpuState, instruction_address), c32(*context, item.pc));
        storeReg(b, *context, state, 15U, c32(*context, item.pc + item.size));
        switch (d.kind) {
        case InstrKind::nop: break;
        case InstrKind::mov: {
            Value* value = c32(*context, d.imm);
            storeReg(b, *context, state, d.rd, value);
            if (d.set_flags) {
                Value* carry = d.immediate_carry_valid
                    ? llvm::ConstantInt::get(Type::getInt1Ty(*context), d.immediate_carry)
                    : b.CreateICmpNE(b.CreateAnd(loadField(b, *context, state, offsetof(CpuState, xpsr)), c32(*context, xpsr_c)), c32(*context, 0));
                writeNzc(b, *context, state, value, carry);
            }
            break;
        }
        case InstrKind::movw: storeReg(b, *context, state, d.rd, c32(*context, d.imm)); break;
        case InstrKind::movt: {
            Value* old = loadReg(b, *context, state, d.rd);
            storeReg(b, *context, state, d.rd, b.CreateOr(b.CreateAnd(old, c32(*context, 0xffffU)), c32(*context, d.imm << 16U)));
            break;
        }
        case InstrKind::add:
        case InstrKind::sub: {
            Value* left = loadReg(b, *context, state, d.rn);
            Value* right = c32(*context, d.imm);
            Value* result = d.kind == InstrKind::add ? b.CreateAdd(left, right) : b.CreateSub(left, right);
            storeReg(b, *context, state, d.rd, result);
            if (d.set_flags) emitAddFlags(b, *context, state, left, right, d.kind == InstrKind::sub);
            break;
        }
        case InstrKind::cmp:
        case InstrKind::cmn: emitAddFlags(b, *context, state, loadReg(b, *context, state, d.rn), c32(*context, d.imm), d.kind == InstrKind::cmp); break;
        case InstrKind::b:
        case InstrKind::bl: {
            if (d.kind == InstrKind::bl) storeReg(b, *context, state, 14U, c32(*context, (item.pc + item.size) | 1U));
            const std::uint32_t target = static_cast<std::uint32_t>(static_cast<std::int64_t>(item.pc) + 4 + d.branch_offset) & ~1U;
            storeReg(b, *context, state, 15U, c32(*context, target));
            break;
        }
        default: break;
        }
        b.CreateBr(next);
        b.SetInsertPoint(next);
    }
    b.CreateRetVoid();
    std::string verifier;
    if (!verifyModule(*module, verifier)) {
        error = "LLVM module verification failed before optimization: " + verifier;
        return {};
    }

    llvm::LoopAnalysisManager loop_analyses;
    llvm::FunctionAnalysisManager function_analyses;
    llvm::CGSCCAnalysisManager cgscc_analyses;
    llvm::ModuleAnalysisManager module_analyses;
    llvm::PassBuilder pass_builder;
    pass_builder.registerModuleAnalyses(module_analyses);
    pass_builder.registerCGSCCAnalyses(cgscc_analyses);
    pass_builder.registerFunctionAnalyses(function_analyses);
    pass_builder.registerLoopAnalyses(loop_analyses);
    pass_builder.crossRegisterProxies(loop_analyses, function_analyses,
                                      cgscc_analyses, module_analyses);
    auto optimization_pipeline = pass_builder.buildPerModuleDefaultPipeline(llvm::OptimizationLevel::O2);
    optimization_pipeline.run(*module, module_analyses);
    if (!verifyModule(*module, verifier)) {
        error = "LLVM module verification failed after optimization: " + verifier;
        return {};
    }

    auto tracker = impl_->jit->getMainJITDylib().createResourceTracker();
    const auto symbol = fn->getName().str();
    {
        std::lock_guard lock(impl_->mutex);
        if (auto addError = impl_->jit->addIRModule(tracker, llvm::orc::ThreadSafeModule(std::move(module), std::move(context)))) {
            error = llvm::toString(std::move(addError));
            llvm::consumeError(tracker->remove());
            return {};
        }
        auto lookedUp = impl_->jit->lookup(symbol);
        if (!lookedUp) {
            error = llvm::toString(lookedUp.takeError());
            llvm::consumeError(tracker->remove());
            return {};
        }
        auto kernel = std::make_unique<NativeJitKernel::Impl>();
        kernel->runtime = impl_;
        kernel->tracker = std::move(tracker);
        kernel->function = lookedUp->toPtr<NativeFunction>();
        return std::shared_ptr<const NativeJitKernel>(new NativeJitKernel(std::move(kernel)));
    }
}

} // namespace fil::cpu
