#include "fil/cpu/jit_llvm.hpp"

#include "fil/cpu/jit.hpp"

#include <llvm/ExecutionEngine/Orc/LLJIT.h>
#include <llvm/ExecutionEngine/Orc/ThreadSafeModule.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/Error.h>
#include <llvm/Support/TargetSelect.h>

#include <atomic>
#include <stdexcept>
#include <utility>

namespace fil::cpu {
namespace {

[[noreturn]] void throwLlvm(llvm::Error error) {
    throw std::runtime_error(llvm::toString(std::move(error)));
}

} // namespace

struct LlvmJitEngine::Impl {
    Impl() {
        if (llvm::InitializeNativeTarget()) {
            throw std::runtime_error("LLVM failed to initialize the native target");
        }
        llvm::InitializeNativeTargetAsmPrinter();
        auto created = llvm::orc::LLJITBuilder().create();
        if (!created) throwLlvm(created.takeError());
        jit = std::move(*created);
    }

    std::unique_ptr<llvm::orc::LLJIT> jit;
    std::atomic<std::uint64_t> next_symbol{0U};
};

LlvmJitEngine::LlvmJitEngine() : impl_(std::make_unique<Impl>()) {}
LlvmJitEngine::~LlvmJitEngine() = default;

LlvmJitEngine::CompiledBlock LlvmJitEngine::compile(
    const std::span<const DecodedInstruction> instructions
) {
    const JitBlockPlan plan = planJitBlock(instructions);
    if (plan.translated_instructions != instructions.size()) {
        throw std::invalid_argument("LLVM JIT input contains a mandatory block boundary");
    }

    auto context = std::make_unique<llvm::LLVMContext>();
    auto module = std::make_unique<llvm::Module>("fil-jit", *context);
    module->setDataLayout(impl_->jit->getDataLayout());
    llvm::IRBuilder<> builder(*context);
    llvm::Type* const i32 = builder.getInt32Ty();
    llvm::Type* const i64 = builder.getInt64Ty();
    llvm::Type* const pointer = builder.getPtrTy();
    llvm::FunctionType* const function_type = llvm::FunctionType::get(
        i64, {pointer, pointer}, false
    );
    const std::string symbol = "fil_jit_block_"
        + std::to_string(impl_->next_symbol.fetch_add(1U));
    llvm::Function* const function = llvm::Function::Create(
        function_type, llvm::Function::ExternalLinkage, symbol, *module
    );
    auto argument = function->arg_begin();
    llvm::Value* const registers = &*argument++;
    llvm::Value* const xpsr = &*argument;
    static_cast<void>(xpsr);
    llvm::BasicBlock* const entry = llvm::BasicBlock::Create(
        *context, "entry", function
    );
    builder.SetInsertPoint(entry);

    const auto registerAddress = [&](const std::uint8_t index) {
        return builder.CreateGEP(
            i32, registers, llvm::ConstantInt::get(i32, index), "reg.addr"
        );
    };
    const auto loadRegister = [&](const std::uint8_t index) {
        return builder.CreateLoad(i32, registerAddress(index), "reg");
    };
    const auto storeRegister = [&](const std::uint8_t index, llvm::Value* value) {
        builder.CreateStore(value, registerAddress(index));
    };

    for (const DecodedInstruction& instruction : instructions) {
        switch (instruction.kind) {
        case InstrKind::movw:
            storeRegister(instruction.rd, llvm::ConstantInt::get(i32, instruction.imm));
            break;
        case InstrKind::movt: {
            llvm::Value* const old = loadRegister(instruction.rd);
            llvm::Value* const low = builder.CreateAnd(old, 0xffffU);
            storeRegister(
                instruction.rd,
                builder.CreateOr(low, instruction.imm << 16U)
            );
            break;
        }
        case InstrKind::add:
        case InstrKind::sub: {
            if (instruction.form != OperandForm::immediate || instruction.set_flags
                || instruction.rd == 15U || instruction.rn == 15U) {
                throw std::invalid_argument("LLVM initial tier supports only flag-free immediate ADD/SUB");
            }
            llvm::Value* const left = loadRegister(instruction.rn);
            llvm::Value* const right = llvm::ConstantInt::get(i32, instruction.imm);
            storeRegister(
                instruction.rd,
                instruction.kind == InstrKind::add
                    ? builder.CreateAdd(left, right)
                    : builder.CreateSub(left, right)
            );
            break;
        }
        case InstrKind::nop:
            break;
        default:
            throw std::invalid_argument("instruction is classified pure but has no LLVM lowering yet");
        }
    }

    builder.CreateRet(llvm::ConstantInt::get(i64, instructions.size()));
    llvm::orc::ThreadSafeModule thread_safe(
        std::move(module), std::move(context)
    );
    if (llvm::Error error = impl_->jit->addIRModule(std::move(thread_safe))) {
        throwLlvm(std::move(error));
    }
    auto found = impl_->jit->lookup(symbol);
    if (!found) throwLlvm(found.takeError());
    return CompiledBlock{
        found->toPtr<BlockFunction>(),
        static_cast<std::uint64_t>(instructions.size()),
        symbol,
    };
}

} // namespace fil::cpu
