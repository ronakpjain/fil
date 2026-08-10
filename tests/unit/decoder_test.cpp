#include "fil/cpu/decoder.hpp"

#include <gtest/gtest.h>

namespace {

TEST(DecoderTest, IdentifiesInstructionWidthsAndValidatesTables) {
    EXPECT_TRUE(!fil::cpu::is32BitThumbPrefix(0xe7ffU)) << "E7FF remains a 16-bit branch";
    EXPECT_TRUE(fil::cpu::is32BitThumbPrefix(0xe800U)) << "E800 introduces a 32-bit encoding";
    EXPECT_TRUE(fil::cpu::is32BitThumbPrefix(0xf000U)) << "F000 introduces a 32-bit encoding";
    EXPECT_TRUE(fil::cpu::is32BitThumbPrefix(0xf800U)) << "F800 introduces a 32-bit encoding";
    EXPECT_TRUE(fil::cpu::decoderTablesHaveNoOverlaps())
        << "decoder mask/value rows do not overlap";
    EXPECT_TRUE(!fil::cpu::decodePatterns16().empty())
        << "16-bit decoder table is externally inspectable";
    EXPECT_TRUE(!fil::cpu::decodePatterns32().empty())
        << "32-bit decoder table is externally inspectable";
}

TEST(DecoderTest, DecodesRepresentativeSixteenBitFamilies) {
    const auto movs = fil::cpu::decode16(0x227fU);
    EXPECT_TRUE(movs && movs->kind == fil::cpu::InstrKind::mov) << "decodes MOVS immediate family";
    EXPECT_TRUE(movs && movs->rd == 2U && movs->imm == 0x7fU && movs->set_flags)
        << "normalizes MOVS operands";

    const auto add = fil::cpu::decode16(0x1888U);
    EXPECT_TRUE(add && add->kind == fil::cpu::InstrKind::add) << "decodes ADD register family";
    EXPECT_TRUE(add && add->rd == 0U && add->rn == 1U && add->rm == 2U)
        << "normalizes ADD registers";

    const auto literal = fil::cpu::decode16(0x4903U);
    EXPECT_TRUE(literal && literal->kind == fil::cpu::InstrKind::ldr) << "decodes literal LDR";
    EXPECT_TRUE(literal && literal->form == fil::cpu::OperandForm::literal && literal->imm == 12U)
        << "scales literal LDR offset";

    const auto store = fil::cpu::decode16(0x60caU);
    EXPECT_TRUE(store && store->kind == fil::cpu::InstrKind::str) << "decodes immediate STR";
    EXPECT_TRUE(store && store->rn == 1U && store->rd == 2U && store->imm == 12U)
        << "scales word STR offset";

    const auto push = fil::cpu::decode16(0xb510U);
    const auto pop = fil::cpu::decode16(0xbd10U);
    EXPECT_TRUE(push && push->register_list == 0x4010U) << "decodes PUSH LR register bit";
    EXPECT_TRUE(pop && pop->register_list == 0x8010U) << "decodes POP PC register bit";

    const auto branch = fil::cpu::decode16(0xd1feU);
    EXPECT_TRUE(branch && branch->condition == fil::cpu::Condition::ne)
        << "decodes conditional branch condition";
    EXPECT_TRUE(branch && branch->branch_offset == -4) << "sign-extends conditional branch offset";

    const auto bx = fil::cpu::decode16(0x4770U);
    EXPECT_TRUE(bx && bx->kind == fil::cpu::InstrKind::bx && bx->rm == 14U) << "decodes BX LR";
}

TEST(DecoderTest, DecodesRepresentativeThirtyTwoBitFamilies) {
    const auto movw = fil::cpu::decode32(0xf241U, 0x2034U);
    const auto movt = fil::cpu::decode32(0xf2c1U, 0x2034U);
    EXPECT_TRUE(movw && movw->kind == fil::cpu::InstrKind::movw && movw->imm == 0x1234U)
        << "decodes MOVW immediate fields";
    EXPECT_TRUE(movt && movt->kind == fil::cpu::InstrKind::movt && movt->imm == 0x1234U)
        << "decodes MOVT immediate fields";

    const auto bl = fil::cpu::decode32(0xf000U, 0xf800U);
    EXPECT_TRUE(bl && bl->kind == fil::cpu::InstrKind::bl && bl->branch_offset == 0)
        << "decodes BL and reconstructs zero displacement";

    const auto ldr = fil::cpu::decode32(0xf8d1U, 0x0020U);
    EXPECT_TRUE(ldr && ldr->kind == fil::cpu::InstrKind::ldr) << "decodes LDR.W immediate family";
    EXPECT_TRUE(ldr && ldr->rn == 1U && ldr->rd == 0U && ldr->imm == 32U)
        << "normalizes LDR.W operands";

    const auto literal_up = fil::cpu::decode32(0xf8dfU, 0x0010U);
    const auto literal_down = fil::cpu::decode32(0xf85fU, 0x0010U);
    EXPECT_TRUE(literal_up && literal_up->form == fil::cpu::OperandForm::literal && literal_up->add)
        << "decodes positive LDR.W literal U bit";
    EXPECT_TRUE(
        literal_down && literal_down->form == fil::cpu::OperandForm::literal && !literal_down->add)
        << "decodes negative LDR.W literal U bit";

    const auto push = fil::cpu::decode32(0xe92dU, 0x41f0U);
    const auto pop = fil::cpu::decode32(0xe8bdU, 0x81f0U);
    EXPECT_TRUE(push && push->kind == fil::cpu::InstrKind::push) << "decodes wide PUSH alias";
    EXPECT_TRUE(pop && pop->kind == fil::cpu::InstrKind::pop) << "decodes wide POP alias";
}

TEST(DecoderTest, DecodesRealG4AndFreeRtosEncodings) {
    const auto orr_imm = fil::cpu::decode32(0xf443U, 0x0370U);
    EXPECT_TRUE(orr_imm && orr_imm->kind == fil::cpu::InstrKind::orr && orr_imm->rn == 3U &&
                orr_imm->rd == 3U && orr_imm->imm == 0x00f00000U)
        << "decodes real SystemInit ORR.W modified immediate";

    const auto cmp_shift = fil::cpu::decode32(0xebb6U, 0x0fa5U);
    EXPECT_TRUE(cmp_shift && cmp_shift->kind == fil::cpu::InstrKind::cmp && cmp_shift->rn == 6U &&
                cmp_shift->rm == 5U && cmp_shift->shift_type == fil::cpu::ShiftType::asr &&
                cmp_shift->shift_amount == 2U)
        << "decodes real CMP.W shifted-register encoding";

    const auto mov_shift = fil::cpu::decode32(0xea4fU, 0x0c93U);
    EXPECT_TRUE(mov_shift && mov_shift->kind == fil::cpu::InstrKind::mov && mov_shift->rd == 12U &&
                mov_shift->rm == 3U && mov_shift->shift_type == fil::cpu::ShiftType::lsr &&
                mov_shift->shift_amount == 2U)
        << "decodes real MOV.W shifted-register encoding";

    const auto addw = fil::cpu::decode32(0xf602U, 0x420cU);
    EXPECT_TRUE(addw && addw->kind == fil::cpu::InstrKind::add && addw->rn == 2U &&
                addw->rd == 2U && addw->imm == 0xc0cU)
        << "decodes real front-driveline ADDW plain immediate";

    const auto register_shift = fil::cpu::decode32(0xfa03U, 0xf000U);
    EXPECT_TRUE(register_shift && register_shift->kind == fil::cpu::InstrKind::lsl &&
                register_shift->rn == 3U && register_shift->rm == 0U && register_shift->rd == 0U)
        << "decodes real register-controlled LSL.W encoding";

    const auto udiv = fil::cpu::decode32(0xfbb5U, 0xf2f6U);
    const auto mls = fil::cpu::decode32(0xfb06U, 0x5212U);
    const auto umull = fil::cpu::decode32(0xfba1U, 0x1303U);
    const auto mla = fil::cpu::decode32(0xfb01U, 0x3002U);
    EXPECT_TRUE(udiv && udiv->kind == fil::cpu::InstrKind::udiv) << "decodes real UDIV encoding";
    EXPECT_TRUE(mls && mls->kind == fil::cpu::InstrKind::mls) << "decodes real MLS encoding";
    EXPECT_TRUE(umull && umull->kind == fil::cpu::InstrKind::umull)
        << "decodes real UMULL encoding";
    EXPECT_TRUE(mla && mla->kind == fil::cpu::InstrKind::mla) << "decodes real MLA encoding";

    const auto clz = fil::cpu::decode32(0xfab2U, 0xf282U);
    const auto ubfx = fil::cpu::decode32(0xf3c3U, 0x1303U);
    EXPECT_TRUE(clz && clz->kind == fil::cpu::InstrKind::clz) << "decodes real CLZ encoding";
    EXPECT_TRUE(ubfx && ubfx->kind == fil::cpu::InstrKind::ubfx && ubfx->shift_amount == 4U &&
                ubfx->imm == 4U)
        << "decodes real UBFX bit range";

    const auto strd = fil::cpu::decode32(0xe96dU, 0xce04U);
    const auto ldrd = fil::cpu::decode32(0xe9ddU, 0x2302U);
    EXPECT_TRUE(strd && strd->kind == fil::cpu::InstrKind::strd && strd->writeback && !strd->add &&
                strd->imm == 16U)
        << "decodes real pre-decrement STRD encoding";
    EXPECT_TRUE(
        ldrd && ldrd->kind == fil::cpu::InstrKind::ldrd && ldrd->rn == 13U && ldrd->imm == 8U)
        << "decodes real LDRD encoding";

    const auto mrs = fil::cpu::decode32(0xf3efU, 0x8305U);
    const auto msr = fil::cpu::decode32(0xf383U, 0x8811U);
    const auto dsb = fil::cpu::decode32(0xf3bfU, 0x8f4fU);
    const auto isb = fil::cpu::decode32(0xf3bfU, 0x8f6fU);
    EXPECT_TRUE(mrs && mrs->kind == fil::cpu::InstrKind::mrs && mrs->imm == 5U)
        << "decodes MRS IPSR";
    EXPECT_TRUE(msr && msr->kind == fil::cpu::InstrKind::msr && msr->imm == 17U)
        << "decodes MSR BASEPRI";
    EXPECT_TRUE(dsb && dsb->kind == fil::cpu::InstrKind::dsb) << "decodes DSB SY";
    EXPECT_TRUE(isb && isb->kind == fil::cpu::InstrKind::isb) << "decodes ISB SY";

    const auto vstm = fil::cpu::decode32(0xed20U, 0x8a10U);
    const auto vldm = fil::cpu::decode32(0xecb0U, 0x8a10U);
    EXPECT_TRUE(vstm && vstm->kind == fil::cpu::InstrKind::vstm && vstm->rd == 16U &&
                vstm->imm == 16U && vstm->writeback)
        << "decodes FreeRTOS VSTMDB s16-s31";
    EXPECT_TRUE(vldm && vldm->kind == fil::cpu::InstrKind::vldm && vldm->rd == 16U &&
                vldm->imm == 16U && vldm->writeback)
        << "decodes FreeRTOS VLDMIA s16-s31";

    const auto vmov_to_s = fil::cpu::decode32(0xee07U, 0x6a90U);
    const auto vmov_to_core = fil::cpu::decode32(0xee17U, 0x3a90U);
    EXPECT_TRUE(vmov_to_s && vmov_to_s->kind == fil::cpu::InstrKind::vmov_core_to_single &&
                vmov_to_s->rd == 15U && vmov_to_s->rn == 6U)
        << "decodes real VMOV from core register to S register";
    EXPECT_TRUE(vmov_to_core && vmov_to_core->kind == fil::cpu::InstrKind::vmov_single_to_core &&
                vmov_to_core->rd == 3U && vmov_to_core->rn == 15U)
        << "decodes real VMOV from S register to core register";

    const auto vcvt_unsigned = fil::cpu::decode32(0xeeb8U, 0x0a67U);
    const auto vcvt_signed = fil::cpu::decode32(0xeef8U, 0x7ae7U);
    const auto vcvt_to_int = fil::cpu::decode32(0xeefdU, 0x7ae7U);
    EXPECT_TRUE(vcvt_unsigned && vcvt_unsigned->kind == fil::cpu::InstrKind::vcvt_f32_u32 &&
                vcvt_unsigned->rd == 0U && vcvt_unsigned->rm == 15U)
        << "decodes real VCVT.F32.U32";
    EXPECT_TRUE(vcvt_signed && vcvt_signed->kind == fil::cpu::InstrKind::vcvt_f32_s32)
        << "decodes real VCVT.F32.S32";
    EXPECT_TRUE(vcvt_to_int && vcvt_to_int->kind == fil::cpu::InstrKind::vcvt_s32_f32)
        << "decodes real VCVT.S32.F32";

    const auto vldr = fil::cpu::decode32(0xed9fU, 0x0a63U);
    const auto vstr = fil::cpu::decode32(0xedc2U, 0x7a1cU);
    const auto vldr_negative = fil::cpu::decode32(0xed1cU, 0x6a03U);
    EXPECT_TRUE(vldr && vldr->kind == fil::cpu::InstrKind::vldr &&
                vldr->form == fil::cpu::OperandForm::literal && vldr->rd == 0U && vldr->imm == 396U)
        << "decodes real PC-relative VLDR";
    EXPECT_TRUE(vstr && vstr->kind == fil::cpu::InstrKind::vstr && vstr->rd == 15U)
        << "decodes real scalar VSTR";
    EXPECT_TRUE(vldr_negative && vldr_negative->kind == fil::cpu::InstrKind::vldr &&
                !vldr_negative->add && vldr_negative->imm == 12U)
        << "keeps negative-offset VLDR distinct from VLDM";

    const auto vadd = fil::cpu::decode32(0xee37U, 0x7a20U);
    const auto vmul = fil::cpu::decode32(0xee27U, 0x7a87U);
    const auto vdiv = fil::cpu::decode32(0xeec6U, 0x7a87U);
    const auto vcmp = fil::cpu::decode32(0xeeb4U, 0x7ac0U);
    const auto vmrs = fil::cpu::decode32(0xeef1U, 0xfa10U);
    EXPECT_TRUE(vadd && vadd->kind == fil::cpu::InstrKind::vadd) << "decodes VADD.F32";
    EXPECT_TRUE(vmul && vmul->kind == fil::cpu::InstrKind::vmul) << "decodes VMUL.F32";
    EXPECT_TRUE(vdiv && vdiv->kind == fil::cpu::InstrKind::vdiv) << "decodes VDIV.F32";
    EXPECT_TRUE(vcmp && vcmp->kind == fil::cpu::InstrKind::vcmp) << "decodes VCMP/VCMPE.F32";
    EXPECT_TRUE(vmrs && vmrs->kind == fil::cpu::InstrKind::vmrs) << "decodes VMRS APSR_nzcv,FPSCR";
}

TEST(DecoderTest, RejectsPrefixesAndReservedEncodings) {
    EXPECT_TRUE(!fil::cpu::decode16(0xf000U)) << "16-bit decoder rejects a wide prefix";
    EXPECT_TRUE(!fil::cpu::decode16(0xde00U))
        << "rejects reserved conditional branch condition 0xE";
    const auto svc = fil::cpu::decode16(0xdf00U);
    EXPECT_TRUE(svc && svc->kind == fil::cpu::InstrKind::svc) << "decodes SVC family distinctly";
    const auto uxtb = fil::cpu::decode16(0xb2f6U);
    EXPECT_TRUE(uxtb && uxtb->kind == fil::cpu::InstrKind::uxtb) << "decodes real UXTB encoding";
    const auto rev16 = fil::cpu::decode16(0xba5bU);
    EXPECT_TRUE(
        rev16 && rev16->kind == fil::cpu::InstrKind::rev16 && rev16->rd == 3U && rev16->rm == 3U)
        << "decodes real REV16 encoding";
    const auto sdiv = fil::cpu::decode32(0xfb93U, 0xf1f2U);
    const auto smull = fil::cpu::decode32(0xfb82U, 0x1203U);
    const auto uadd8 = fil::cpu::decode32(0xfa85U, 0xf547U);
    const auto sel = fil::cpu::decode32(0xfaa3U, 0xf587U);
    EXPECT_TRUE(sdiv && sdiv->kind == fil::cpu::InstrKind::sdiv) << "decodes real SDIV encoding";
    EXPECT_TRUE(smull && smull->kind == fil::cpu::InstrKind::smull)
        << "decodes real SMULL encoding";
    EXPECT_TRUE(uadd8 && uadd8->kind == fil::cpu::InstrKind::uadd8)
        << "decodes real UADD8 encoding";
    EXPECT_TRUE(sel && sel->kind == fil::cpu::InstrKind::sel) << "decodes real SEL encoding";
    EXPECT_TRUE(!fil::cpu::decode16(0xb400U)) << "rejects empty PUSH register list";
    EXPECT_TRUE(!fil::cpu::decode16(0x4701U)) << "rejects reserved branch-exchange low bits";
    EXPECT_TRUE(!fil::cpu::decode16(0xbf10U))
        << "rejects unsupported hint rather than treating it as IT";
    EXPECT_TRUE(!fil::cpu::decode32(0xeef1U, 0x0a10U))
        << "keeps unsupported general VMRS destination strict";
}

} // namespace
