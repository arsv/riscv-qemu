#include "qemu/osdep.h"
#include "cpu.h"
#include "translate.h"
#include "qemu-common.h"

#include "tcg-op.h"
#include "qemu/log.h"
#include "qemu/bitops.h"
#include "exec/cpu_ldst.h"

#include "exec/helper-proto.h"
#include "exec/helper-gen.h"

#include "trace-tcg.h"
#include "exec/log.h"

/* Most functions here get disas context as their first argument.
   It's always the same one, allocated in gen_intermediate_code()
   once for each invocation and thus for each TB. RISC-V lacks most
   features that require DC (delay branches and such) so it only
   tracks PC of the insn being translated, and jump signal.

   Any PC-relative arithmetics here must use dc->pc, not env->pc from
   RISCVCPUState structure as that one does not get updated from within
   a running TB and may be off. Any jumps must update env->pc to the value
   is it supposed to have at that point and set the jump flag to interrupt
   the translation loop and end current TB. */

typedef struct DisasContext {
    TranslationBlock *tb;
    target_ulong pc;
    bool jump;
} DisasContext;

#define DC DisasContext* dc

/* TCG references to CPU registers */

static TCGv_env cpu_env;
static TCGv cpu_pc;
static TCGv cpu_gpr[32];
static TCGv cpu_fpr[32];

/* FIXME: it is wrong to assume sizeof(fpr) == sizeof(gpr), in particular
   RV32 is likely to have 64-bit floats (RV32D), but for now the implementation
   is limited to the easiest case of RV64 w/ 64-bit floats.

   For RV32D, TCGv points to TARGET_LONG_BITS (=32) value, a second type
   will be needed, pointing to TARGET_FLOAT_LONG (=64) value. This is partially
   done in FP helpers with gpv and fpv. */

#include "exec/gen-icount.h"

#define BITFIELD(src, end, start) \
            (((src) >> start) & ((1 << (end - start + 1)) - 1))
#define SIGNBIT ((target_long)1 << (TARGET_LONG_BITS - 1))

/* Register x0 is a zero-sink, always reads as 0 and writes there are ignored.
   Within qemu, it's an allocated TCGv holding 0, used directly for any read
   access and guarded against writes.

   Most insns allocate a throwaway temp TCGv for x0 writes.
   Short-cut to nop with rd=0 is not always possible since writes to x0
   still need to generate exceptions whenever other insn fields are wrong. */

static const char* const riscv_gprnames[] = {
     "x0",  "x1",  "x2",  "x3",  "x4",  "x5",  "x6",  "x7",
    " x8",  "x9", "x10", "x11", "x12", "x13", "x14", "x15",
    "x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23",
    "x24", "x25", "x26", "x27", "x28", "x29", "x30", "x31",
};

static const char* const riscv_fprnames[] = {
     "f0",  "f1",  "f2",  "f3",  "f4",  "f5",  "f6",  "f7",
    " f8",  "f9", "f10", "f11", "f12", "f13", "f14", "f15",
    "f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
    "f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31",
};

void riscv_translate_init(void)
{
    static bool initialized = 0;

    if(!tcg_enabled())
        return;
    if(initialized)
        return;
    initialized = 1;

    cpu_env = tcg_global_reg_new_ptr(TCG_AREG0, "env");
    tcg_ctx.tcg_env = cpu_env;

    cpu_pc = tcg_global_mem_new(cpu_env, offsetof(CPURISCVState, pc), "pc");

    int i;
    for(i = 0; i < 32; i++)
        cpu_gpr[i] = tcg_global_mem_new(cpu_env,
                        offsetof(CPURISCVState, gpr[i]),
                        riscv_gprnames[i]);
    for(i = 0; i < 32; i++)
        cpu_fpr[i] = tcg_global_mem_new(cpu_env,
                        offsetof(CPURISCVState, fpr[i]),
                        riscv_fprnames[i]);
}

void restore_state_to_opc(CPURISCVState *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->pc = data[0];
}

/* All the stuff below this line is gen_intermediate_code() */

static int tblock_max_insns(struct TranslationBlock *tb)
{
    int max = tb->cflags & CF_COUNT_MASK;

    if(!max)
        max = CF_COUNT_MASK;
    if(max > TCG_MAX_INSNS)
        max = TCG_MAX_INSNS;

    return max;
}

static TCGv temp_new_rsum(TCGv vs, int32_t imm)
{
    TCGv vt = tcg_temp_new();
    tcg_gen_mov_tl(vt, vs);
    tcg_gen_addi_tl(vt, vt, imm);
    return vt;
}

static TCGv temp_new_ext32s(TCGv vs)
{
    TCGv vx = tcg_temp_new();
    tcg_gen_ext32s_tl(vx, vs);
    return vx;
}

static TCGv temp_new_ext32u(TCGv vs)
{
    TCGv vx = tcg_temp_new();
    tcg_gen_ext32u_tl(vx, vs);
    return vx;
}

static TCGv temp_new_andi(TCGv vs, target_long mask)
{
    TCGv vr = tcg_temp_new();
    tcg_gen_andi_tl(vr, vs, mask);
    return vr;
}

static void gen_exception(DC, unsigned int excp)
{
    TCGv_i32 tmp = tcg_const_i32(excp);
    tcg_gen_movi_tl(cpu_pc, dc->pc);
    gen_helper_exception(cpu_env, tmp);
    tcg_temp_free_i32(tmp);
}

static void gen_illegal(DC)
{
    gen_exception(dc, EXCP_ILLEGAL);
}

/* Load upper immediate: rd = (imm << 12)
   Since imm is 31:12 in insn, there is no need to shift it. */

static void gen_lui(DC, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    uint32_t uimm = insn & 0xFFFFF000;
    TCGv vd = cpu_gpr[rd];

    if(!rd) return; /* write to x0 is nop */

    tcg_gen_movi_tl(vd, uimm);
    tcg_gen_ext32s_tl(vd, vd);
}

/* Add upper intermediate to pc: rd = pc + (imm << 12)
   Again, no need to shift imm. */

static void gen_auipc(DC, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    uint32_t imm = insn & 0xFFFFF000;
    TCGv vd = cpu_gpr[rd];

    if(!rd) return; /* write to x0 is nop */

    tcg_gen_movi_tl(vd, imm);
    tcg_gen_ext32s_tl(vd, vd);
    /* cpu_pc is not updated on each tick and may be off */
    tcg_gen_add_tl(vd, vd, tcg_const_tl(dc->pc));
}

/* Shift Left/Right Logical/Arithm by Immediate. */

static void gen_slli(DC, TCGv vd, TCGv vs, unsigned shamt, unsigned flags)
{
    if(flags)
        gen_illegal(dc);
    else
        tcg_gen_shli_tl(vd, vs, shamt);
}

static void gen_srxi(DC, TCGv vd, TCGv vs, unsigned shamt, unsigned flags)
{
    if(flags & ~(1<<5))
        gen_illegal(dc);
    else if(flags & (1<<5))
        tcg_gen_sari_tl(vd, vs, shamt);
    else
        tcg_gen_shri_tl(vd, vs, shamt);
}

static void gen_slti(TCGv vd, TCGv vs, int32_t imm)
{
    tcg_gen_setcondi_tl(TCG_COND_LT,  vd, vs, imm);
}

static void gen_sltiu(TCGv vd, TCGv vs, int32_t imm)
{
    /* Per spec, imm is signed even in *U case */
    tcg_gen_setcondi_tl(TCG_COND_LTU, vd, vs, imm);
}

/* Arithmetics with immediate: rd = rs op imm;
   ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI. */

static void gen_opimm(DC, uint32_t insn)
{
    int32_t imm = ((int32_t)insn) >> 20;
    unsigned rs = BITFIELD(insn, 19, 15);
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned shamt = BITFIELD(insn, 24, 20);
    unsigned flags = BITFIELD(insn, 31, 25);

    TCGv vd = rd ? cpu_gpr[rd] : tcg_temp_new();
    TCGv vs = cpu_gpr[rs];

    switch(BITFIELD(insn, 14, 12)) {
        case /* 000 */ 0: tcg_gen_addi_tl(vd, vs, imm); break;
        case /* 100 */ 4: tcg_gen_xori_tl(vd, vs, imm); break;
        case /* 110 */ 6: tcg_gen_ori_tl(vd, vs, imm); break;
        case /* 111 */ 7: tcg_gen_andi_tl(vd, vs, imm); break;
        case /* 010 */ 2: gen_slti(vd, vs, imm); break;
        case /* 011 */ 3: gen_sltiu(vd, vs, imm); break;
        case /* 001 */ 1: gen_slli(dc, vd, vs, shamt, flags); break;
        case /* 101 */ 5: gen_srxi(dc, vd, vs, shamt, flags); break;
        default: gen_illegal(dc);
    }

    if(!rd) tcg_temp_free(vd);
}

/* Like OPIMM but with 32-bit words on RV64; ADDIW, SLLIW, SRLIW, SRAIW */

static void gen_opimm32(DC, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned rs = BITFIELD(insn, 19, 15);
    int32_t imm = ((int32_t)insn >> 20);      /* ADDIW only */
    unsigned shamt = BITFIELD(insn, 24, 20);  /* SLLIW, SRLIW, SRAIW */
    unsigned flags = BITFIELD(insn, 31, 25);  /* SLLIW, SRLIW, SRAIW */

    TCGv vd = rd ? cpu_gpr[rd] : tcg_temp_new();
    TCGv vs = cpu_gpr[rs];

    switch(BITFIELD(insn, 14, 12)) {
        case /* 000 */ 0: tcg_gen_addi_tl(vd, vs, imm); break;
        case /* 001 */ 1: gen_slli(dc, vd, vs, shamt, flags); break;
        case /* 101 */ 5: gen_srxi(dc, vd, vs, shamt, flags); break;
        default: gen_illegal(dc); goto out;
    }

    tcg_gen_ext32s_tl(vd, vd);

out:
    if(!rd) tcg_temp_free(vd);
}

/* Logical SLR and arithm SLA shift right: rd = rs1 << rs2
   RISC-V apparently *ignores* high bits in shift amount register,
   so (v >> 75) == (v >> (75 % 64)) == (v >> 11) */

static void gen_srl(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv shamt = temp_new_andi(vs2, TARGET_LONG_BITS - 1);
    tcg_gen_shr_tl(vd, vs1, shamt);
    tcg_temp_free(shamt);
}

static void gen_sra(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv shamt = temp_new_andi(vs2, TARGET_LONG_BITS - 1);
    tcg_gen_sar_tl(vd, vs1, shamt);
    tcg_temp_free(shamt);
}

static void gen_sll(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv shamt = tcg_temp_new();
    tcg_gen_andi_tl(shamt, vs2, TARGET_LONG_BITS - 1);
    tcg_gen_shl_tl(vd, vs1, shamt);
    tcg_temp_free(shamt);
}

/* Multiplying tl x tl yields 2tl result. MUL returns the lower tl,
   MUL*H* variants return higher tl for signed x signed (default),
   unsigned x unsigned (U) and signed x unsigned (SU) cases. */

static void gen_mul(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv sink = tcg_temp_new();
    tcg_gen_muls2_tl(vd, sink, vs1, vs2);
    tcg_temp_free(sink);
}

static void gen_mulh(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv sink = tcg_temp_new();
    tcg_gen_muls2_tl(sink, vd, vs1, vs2);
    tcg_temp_free(sink);
}

static void gen_mulhu(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv sink = tcg_temp_new();
    tcg_gen_mulu2_tl(sink, vd, vs1, vs2);
    tcg_temp_free(sink);
}

/* Signed x unsigned case, not implemented in TCG.
   Workaround: (-s * u) = -(s * u) = ~(s * u) + 1.
   The final addition is two-word long. */

static void gen_mulhsu(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGLabel* skip = gen_new_label();
    TCGLabel* done = gen_new_label();

    TCGv sign = tcg_temp_local_new();
    tcg_gen_andi_tl(sign, vs1, SIGNBIT);
    tcg_gen_brcondi_tl(TCG_COND_NE, sign, 0, skip);
    tcg_temp_free(sign);

    /* positive vs1 case */
    TCGv sink = tcg_temp_local_new();
    tcg_gen_mulu2_tl(sink, vd, vs1, vs2);
    tcg_temp_free(sink);
    tcg_gen_br(done);

    gen_set_label(skip);

    /* negative vs1 case */
    TCGv pos1 = tcg_temp_new();
    tcg_gen_neg_tl(pos1, vs1);
    TCGv resh = tcg_temp_new();
    TCGv resl = tcg_temp_new();
    tcg_gen_mulu2_tl(resl, resh, pos1, vs2);
    tcg_temp_free(pos1);

    /* negate resl and add 1 */
    TCGv notresl = tcg_temp_new();
    tcg_gen_xori_tl(notresl, notresl, -1);    /* notresl = ~resl */
    tcg_gen_addi_tl(resl, notresl, 1);        /* resl = ~resl + 1 */

    /* set vd = ~resh + carry */
    tcg_gen_xori_tl(vd, resh, -1);            /* vd = ~resh */
    tcg_gen_brcond_tl(TCG_COND_LE, notresl, resl, done);
    tcg_gen_addi_tl(vd, vd, 1);               /* vd = vd + 1 */

    gen_set_label(done);

    tcg_temp_free(resh);
    tcg_temp_free(resl);
    tcg_temp_free(notresl);
}

/* RISC-V signals division errors (div by zero and overflow)
   by returning certain special values in rd. The checks
   are common but the values are different for DIVs and REMs. */

/* if(rs2 == 0) { rd = mark; goto done; } */

static void gen_div_zerocheck(TCGLabel* done,
        TCGv vd, TCGv vs1, TCGv vs2, int op_is_rem)
{
    TCGLabel* skip = gen_new_label();

    tcg_gen_brcondi_tl(TCG_COND_NE, vs2, 0, skip);

    if(op_is_rem)
        tcg_gen_mov_tl(vd, vs1);
    else
        tcg_gen_movi_tl(vd, -1);

    tcg_gen_br(done);
    gen_set_label(skip);
}

/* if(rs1 == signbit && rs2 == -1) { rd = mark; goto done; } */

static void gen_div_overcheck(TCGLabel* done,
        TCGv vd, TCGv vs1, TCGv vs2, int op_is_rem)
{
    TCGLabel* skip = gen_new_label();

    TCGv c1 = tcg_temp_new();
    TCGv c2 = tcg_temp_new();

    tcg_gen_setcondi_tl(TCG_COND_EQ, c1, vs2, -1);
    tcg_gen_setcondi_tl(TCG_COND_EQ, c2, vs1, SIGNBIT);
    tcg_gen_and_tl(c1, c1, c2);
    tcg_gen_brcondi_tl(TCG_COND_EQ, c1, 0, skip);

    tcg_temp_free(c1);
    tcg_temp_free(c2);

    if(op_is_rem)
        tcg_gen_movi_tl(vd, 0);
    else
        tcg_gen_mov_tl(vd, vs1);

    tcg_gen_br(done);
    gen_set_label(skip);
}

/* Signed/unsigned DIV and REM: vd = vs1 / vs2 */

static void gen_div(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vs1, vs2, 0);
    gen_div_overcheck(done, vd, vs2, vs2, 0);
    tcg_gen_div_tl(vd, vs1, vs2);

    gen_set_label(done);
}

static void gen_divu(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vs1, vs2, 0);
    tcg_gen_divu_tl(vd, vs1, vs2);

    gen_set_label(done);
}

static void gen_rem(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vs1, vs2, 1);
    gen_div_overcheck(done, vd, vs2, vs2, 1);
    tcg_gen_rem_tl(vd, vs1, vs2);

    gen_set_label(done);
}

static void gen_remu(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vs1, vs2, 1);
    tcg_gen_remu_tl(vd, vs1, vs2);

    gen_set_label(done);
}

/* 3-bit func field is not enough to encode all possible funcs
   for OP and OP32 instructions, so two extra bits are spilled
   into the upper imm field of insn. To simplify switch()es,
   the bits that matter are squashed back together in the same
   order they come in insn: amfff, a = arithm, m = multiply,
   and fff the original funct3 bits. */

static unsigned rvop_extfunc(uint32_t insn)
{
    unsigned func = BITFIELD(insn, 14, 12); /* 3 bits initially */
    unsigned flags = BITFIELD(insn, 31, 25);

    if(flags & ~((1<<5) | (1<<0)))
        return -1;        /* force default case in switches */
    if(flags & (1<<0))
        func |= (1<<3);   /* multiply */
    if(flags & (1<<5))
        func |= (1<<4);   /* arithm */

    return func;
}

/* Too long; didn't fit within the 80c. Set-to less-than(-unsigned). */

static void gen_slt(TCGv vd, TCGv vs1, TCGv vs2)
{
    tcg_gen_setcond_tl(TCG_COND_LT, vd, vs1, vs2);
}

static void gen_sltu(TCGv vd, TCGv vs1, TCGv vs2)
{
    tcg_gen_setcond_tl(TCG_COND_LTU, vd, vs1, vs2);
}

/* Register arithmetics: rd = rs1 op rs2
   ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND. */

static void gen_op(DC, uint32_t insn)
{
    unsigned rs2 = BITFIELD(insn, 24, 20);
    unsigned rs1 = BITFIELD(insn, 19, 15);
    unsigned rd = BITFIELD(insn, 11, 7);

    TCGv vd = rd ? cpu_gpr[rd] : tcg_temp_local_new();
    TCGv vs1 = cpu_gpr[rs1];
    TCGv vs2 = cpu_gpr[rs2];

    switch(rvop_extfunc(insn)) {
        case /* 00.000 */ 0: tcg_gen_add_tl(vd, vs1, vs2); break;
        case /* 10.000 */16: tcg_gen_sub_tl(vd, vs1, vs2); break;
        case /* 00.100 */ 4: tcg_gen_xor_tl(vd, vs1, vs2); break;
        case /* 00.110 */ 6: tcg_gen_or_tl(vd, vs1, vs2); break;
        case /* 00.111 */ 7: tcg_gen_and_tl(vd, vs1, vs2); break;
        case /* 00.010 */ 2: gen_slt(vd, vs1, vs2); break;
        case /* 00.011 */ 3: gen_sltu(vd, vs1, vs2); break;
        case /* 00.001 */ 1: gen_sll(vd, vs1, vs2); break;
        case /* 00.101 */ 5: gen_srl(vd, vs1, vs2); break;
        case /* 10.101 */21: gen_sra(vd, vs1, vs2); break;
        case /* 01.000 */ 8: gen_mul(vd, vs1, vs2); break;
        case /* 01.001 */ 9: gen_mulh(vd, vs1, vs2); break;
        case /* 01.010 */10: gen_mulhsu(vd, vs1, vs2); break;
        case /* 01.011 */11: gen_mulhu(vd, vs1, vs2); break;
        case /* 01.100 */12: gen_div(vd, vs1, vs2); break;
        case /* 01.101 */13: gen_divu(vd, vs1, vs2); break;
        case /* 01.110 */14: gen_rem(vd, vs1, vs2); break;
        case /* 01.111 */15: gen_remu(vd, vs1, vs2); break;
        default: gen_illegal(dc);
    }

    if(!rd) tcg_temp_free(vd);
}

static void gen_addw(TCGv vd, TCGv vs1, TCGv vs2)
{
    tcg_gen_add_tl(vd, vs1, vs2);
    tcg_gen_ext32s_tl(vd, vd);
}

static void gen_subw(TCGv vd, TCGv vs1, TCGv vs2)
{
    tcg_gen_sub_tl(vd, vs1, vs2);
    tcg_gen_ext32s_tl(vd, vd);
}

static void gen_sllw(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv shamt = temp_new_andi(vs2, 31);

    tcg_gen_shl_tl(vd, vs1, shamt);
    tcg_gen_ext32s_tl(vd, vd);

    tcg_temp_free(shamt);
}

static void gen_srlw(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv vx = temp_new_ext32u(vs1);
    TCGv shamt = temp_new_andi(vs2, 31);

    tcg_gen_shr_tl(vd, vx, shamt);
    tcg_gen_ext32s_tl(vd, vd);

    tcg_temp_free(shamt);
    tcg_temp_free(vx);
}

static void gen_sraw(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv vx = temp_new_ext32s(vs1);
    TCGv shamt = temp_new_andi(vs2, 31);

    tcg_gen_sar_tl(vd, vx, shamt);
    tcg_gen_ext32s_tl(vd, vd);

    tcg_temp_free(shamt);
    tcg_temp_free(vx);
}

static void gen_mulw(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv sink = tcg_temp_new();

    /* no need to sign-extend anything there */
    tcg_gen_muls2_tl(vd, sink, vs1, vs2);
    tcg_gen_ext32s_tl(vd, vd);

    tcg_temp_free(sink);
}

static void gen_divw(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv vx1 = temp_new_ext32s(vs1);
    TCGv vx2 = temp_new_ext32s(vs2);
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vx1, vx2, 0);
    /* gen_div_overcheck not needed, DIVW cannot overflow? */
    tcg_gen_div_tl(vd, vx1, vx2);
    tcg_gen_ext32s_tl(vd, vd);

    tcg_temp_free(vx2);
    tcg_temp_free(vx1);

    gen_set_label(done);
}

static void gen_divuw(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv vx1 = temp_new_ext32u(vs1);
    TCGv vx2 = temp_new_ext32u(vs2);
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vx1, vx2, 0);
    tcg_gen_div_tl(vd, vx1, vx2);
    tcg_gen_ext32u_tl(vd, vd);

    tcg_temp_free(vx2);
    tcg_temp_free(vx1);

    gen_set_label(done);
}

static void gen_remw(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv vx1 = temp_new_ext32s(vs1);
    TCGv vx2 = temp_new_ext32s(vs2);
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vx1, vx2, 0);
    /* gen_div_overcheck not needed, DIVW cannot overflow? */
    tcg_gen_rem_tl(vd, vx1, vx2);
    tcg_gen_ext32s_tl(vd, vd);

    tcg_temp_free(vx2);
    tcg_temp_free(vx1);

    gen_set_label(done);
}

static void gen_remuw(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv vx1 = temp_new_ext32u(vs1);
    TCGv vx2 = temp_new_ext32u(vs2);
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vx1, vx2, 0);
    tcg_gen_rem_tl(vd, vx1, vx2);
    tcg_gen_ext32s_tl(vd, vd); /* sign-extend even in U case! */

    tcg_temp_free(vx2);
    tcg_temp_free(vx1);

    gen_set_label(done);
}

/* Like OP but with 32-bit words on RV64.

   Register handling is not uniform here, some insns sign-extend,
   some zero-extend and some do not care at all. So each insn does
   it in its own function. */

static void gen_op32(DC, uint32_t insn)
{
    unsigned rs2 = BITFIELD(insn, 24, 20);
    unsigned rs1 = BITFIELD(insn, 19, 15);
    unsigned rd = BITFIELD(insn, 11, 7);

    TCGv vd = rd ? cpu_gpr[rd] : tcg_temp_local_new();
    TCGv vs1 = cpu_gpr[rs1];
    TCGv vs2 = cpu_gpr[rs2];

    switch(rvop_extfunc(insn)) {
        case /* 00.000 */ 0: gen_addw(vd, vs1, vs2); break;
        case /* 10.000 */16: gen_subw(vd, vs1, vs2); break;
        case /* 00.001 */ 1: gen_sllw(vd, vd, vs2); break;
        case /* 00.101 */ 5: gen_srlw(vd, vd, vs2); break;
        case /* 10.101 */21: gen_sraw(vd, vd, vs2); break;
        case /* 01.000 */ 8: gen_mulw(vd, vs1, vs2); break;
        case /* 01.100 */12: gen_divw(vd, vs1, vs2); break;
        case /* 01.101 */13: gen_divuw(vd, vs1, vs2); break;
        case /* 01.110 */14: gen_remw(vd, vs1, vs2); break;
        case /* 01.111 */15: gen_remuw(vd, vs1, vs2); break;
        default: gen_illegal(dc);
    }

    if(!rd) tcg_temp_free(vd);
}

/* Jump And Link: rd = pc, pc = pc + imm

   Trying to jump (set PC to) a mis-aligned address always results
   in exception. Thus PC is always properly aligned on real hw,
   and it is only necessary to check imm even if the spec demands
   the final address to be aligned.

   The emulation follows this assumption, and does not check PC.
   If somehow PC gets mis-aligned, like for instance via linux-user
   ELF loader setting it that way, it remains so without affecting
   the code being executed. The lower bit(s) of PC are effectively
   ignored.

   This is apparently consistent with Spike behavior, and with
   the idea of all RISC-V code being position independent by design.

   The lowest bit of imm is not encoded, and assumed to be zero.
   Bit 1 may be allowed to be non-zero if two-byte opcodes are
   supported (extension C) but they are not at this point.
   With extension C support enabled, JAL never generates any
   exceptions at all. */

static void gen_jal(DC, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    uint32_t imm =
            (BITFIELD(insn, 31, 31) << 20) |
            (BITFIELD(insn, 19, 12) << 12) |
            (BITFIELD(insn, 20, 20) << 11) |
            (BITFIELD(insn, 30, 21) << 1);

    if(imm % 3) {
        gen_illegal(dc);
        return;
    }

    if(rd) /* JAL x0 is a plain jump */
        tcg_gen_movi_tl(cpu_gpr[rd], dc->pc + 4);

    tcg_gen_movi_tl(cpu_pc, dc->pc + imm);
    tcg_gen_exit_tb(0);

    dc->jump = true;
}

/* Jump And Link Register: rd = pc, pc += rs + imm

   Indirect jump has the same alignment issues as JAL.
   The spec explicitly demands dropping bit 0 from the calculated
   target address (mask -2 below) and non-zero bit 1 only matters
   if extesion C is not supported. With extension C enabled, JALR
   never generates any exceptions.

   PC is again assumed to be pre-aligned. */

static void gen_jalr(DC, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned rs = BITFIELD(insn, 19, 15);
    int32_t imm = ((int32_t)insn) >> 20;
    /* func field (14:12) is apparently ignored? */

    if(rd) /* JALR x0 is a plain indirect jump */
        tcg_gen_movi_tl(cpu_gpr[rd], dc->pc + 4);

    TCGv va = tcg_temp_local_new();

    if(rs)
        tcg_gen_mov_tl(va, cpu_gpr[rs]);
    else
        tcg_gen_movi_tl(va, imm);
    if(rs && imm)
        tcg_gen_addi_tl(va, va, imm);

    tcg_gen_andi_tl(va, va, -2); /* clear bit 0 in va */

    /* Check for address misalignment */
    TCGv vx = tcg_temp_local_new();
    TCGLabel* skip = gen_new_label();

    tcg_gen_andi_tl(vx, va, -3); /* mask out two lower bits of va */
    tcg_gen_brcond_tl(TCG_COND_EQ, vx, va, skip);
    gen_illegal(dc); /* should be misalignment */

    gen_set_label(skip);

    /* We're clear, set PC */
    tcg_gen_add_tl(cpu_pc, cpu_pc, va);
    tcg_gen_exit_tb(0);
    dc->jump = true;

    tcg_temp_free(vx);
    tcg_temp_free(va);
}

/* Conditional branch: if(rs1 op rs2) pc = pc + imm
   BEQ, BNE, BLT, BLTU, BGE, BGEU.

   The actual TCG code generated is

                brcond  !cond, rs1, rs2, skip
                movi    pc, (dc.pc + imm)
                exit
        skip:   ...

   Note the condition is *inverse* of what RISC-V opcode would imply!

   See JAL/JALR comments on target address (mis-)alignment handling. */

static void gen_branch(DC, uint32_t insn)
{
    unsigned rs1 = BITFIELD(insn, 19, 15);
    unsigned rs2 = BITFIELD(insn, 24, 20);
    int16_t imm =
            (BITFIELD(insn, 11,  8) <<  1) |        /*  4:1 */
            (BITFIELD(insn, 30, 25) <<  5) |        /* 10:5 */
            (BITFIELD(insn,  7,  7) << 11) |        /* 11 */
            (((((int32_t)insn) >> 30) & 1) << 12);  /* 12, sign bit */

    if(imm % 3) {
        gen_illegal(dc);
        return;
    }

    TCGv vs1 = cpu_gpr[rs1];
    TCGv vs2 = cpu_gpr[rs2];
    TCGLabel* l = gen_new_label();

    switch(BITFIELD(insn, 14, 12)) {
        case /* 000 */ 0: tcg_gen_brcond_tl(TCG_COND_NE, vs1, vs2, l); break;
        case /* 001 */ 1: tcg_gen_brcond_tl(TCG_COND_EQ, vs1, vs2, l); break;
        case /* 100 */ 4: tcg_gen_brcond_tl(TCG_COND_GE, vs1, vs2, l); break;
        case /* 101 */ 5: tcg_gen_brcond_tl(TCG_COND_LT, vs1, vs2, l); break;
        case /* 110 */ 6: tcg_gen_brcond_tl(TCG_COND_GEU, vs1, vs2, l); break;
        case /* 111 */ 7: tcg_gen_brcond_tl(TCG_COND_LTU, vs1, vs2, l); break;
        default: gen_illegal(dc); return;
    }

    tcg_gen_movi_tl(cpu_pc, dc->pc + imm);
    tcg_gen_exit_tb(0);

    gen_set_label(l);
}

/* Memory load: rd = [rs + imm]; LB, LH, LW, LD, LBU, LHU */

static void gen_load(DC, uint32_t insn)
{
    int32_t imm = ((int32_t)insn) >> 20;
    unsigned rs = BITFIELD(insn, 19, 15);
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned memidx = 0;   /* mmu, always 0 in linux-user mode */

    TCGv va = imm ? temp_new_rsum(cpu_gpr[rs], imm) : cpu_gpr[rs];
    TCGv vd = rd ? cpu_gpr[rd] : tcg_temp_new();

    switch(BITFIELD(insn, 14, 12)) {
        case /* 000 */ 0: tcg_gen_qemu_ld8s(vd, va, memidx); break;
        case /* 001 */ 1: tcg_gen_qemu_ld16s(vd, va, memidx); break;
        case /* 010 */ 2: tcg_gen_qemu_ld32s(vd, va, memidx); break;
        case /* 011 */ 3: tcg_gen_qemu_ld64(vd, va, memidx); break;
        case /* 100 */ 4: tcg_gen_qemu_ld8u(vd, va, memidx); break;
        case /* 101 */ 5: tcg_gen_qemu_ld16u(vd, va, memidx); break;
        case /* 110 */ 6: tcg_gen_qemu_ld32u(vd, va, memidx); break;
        default: gen_illegal(dc);
    }

    if(!rd) tcg_temp_free(vd);
    if(imm) tcg_temp_free(va);
}

/* Memory store: [rs1 + imm] = rs2; SB, SH, SW, SD. */

static void gen_store(DC, uint32_t insn)
{
    unsigned rs1 = BITFIELD(insn, 19, 15);  /* address */
    unsigned rs2 = BITFIELD(insn, 24, 20);  /* data to store */
    int32_t imm = (insn & 0x1F) |           /* 5 lower bits from insn */
           (((int32_t)insn >> 20) & 0x1F);  /* 31:25 and 5 clear bits */
    unsigned memidx = 0;   /* mmu, always 0 in linux-user mode */

    TCGv va = imm ? temp_new_rsum(cpu_gpr[rs1], imm) : cpu_gpr[rs1];
    TCGv vs = cpu_gpr[rs2];

    switch(BITFIELD(insn, 14, 12)) {
        case /* 000 */ 0: tcg_gen_qemu_st8(vs, va, memidx); break;
        case /* 001 */ 1: tcg_gen_qemu_st16(vs, va, memidx); break;
        case /* 010 */ 2: tcg_gen_qemu_st32(vs, va, memidx); break;
        case /* 011 */ 3: tcg_gen_qemu_st64(vs, va, memidx); break;
        default: gen_illegal(dc);
    }

    if(imm) tcg_temp_free(va);
}

/* Memory FENCE and FENCE.I with i-cache flush.
   No-ops for now; rd, rs and imm ignored as per spec. */

static void gen_miscmem(DC, uint32_t insn)
{
    switch(BITFIELD(insn, 14, 12))
    {
        case 0: /* FENCE */
        case 1: /* FENCE.I */
            break;
        default: gen_illegal(dc);
    }
}

/* Privileged insns, plus ECALL and EBREAK which may be used in U-level. */

static void gen_priv(DC, uint32_t insn)
{
    switch(BITFIELD(insn, 31, 20)) {
        case 0: gen_exception(dc, EXCP_SYSCALL); break;
        case 1: /* EBREAK, not implemented */
        default: gen_illegal(dc);
    }
}

/* Syscalls and CSR ops. These are actually two completely unrelated
   instructions sharing a single opcode.

   CSR handling is relegated to a helper. CSR ops are presumably rare,
   most are inherently slow anyway, and the code is complex enough to
   make TCG implementation pointless. */

static void gen_system(DC, uint32_t insn)
{
    switch(BITFIELD(insn, 14, 12)) {
        case /* 000 */ 0: gen_priv(dc, insn); return;
        case /* 100 */ 4: gen_illegal(dc); return;
    }

    /* CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI */
    TCGv_i32 temp = tcg_const_i32(insn);
    gen_helper_csr(cpu_env, temp);
    tcg_temp_free_i32(temp);
}

/* Like LOAD but writes to fpr instead of gpr.
   And f0 is a regular register, not a sink. */

static void gen_loadfp(DC, uint32_t insn)
{
    int imm = ((int32_t)insn >> 20);
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned rs = BITFIELD(insn, 19, 15);
    unsigned memidx = 0;   /* mmu, always 0 in linux-user mode */

    TCGv va = imm ? temp_new_rsum(cpu_gpr[rs], imm) : cpu_gpr[rs];
    TCGv vd = cpu_fpr[rd];

    switch(BITFIELD(insn, 14, 12)) {
        case /* 010 */ 2: tcg_gen_qemu_ld32u(vd, va, memidx); break;
        case /* 011 */ 3: tcg_gen_qemu_ld64(vd, va, memidx); break;
        default: gen_illegal(dc);
    }

    if(imm) tcg_temp_free(va);
}

static void gen_storefp(DC, uint32_t insn)
{
    unsigned rs1 = BITFIELD(insn, 19, 15);  /* address */
    unsigned rs2 = BITFIELD(insn, 24, 20);  /* fpr to store */
    int32_t imm = (insn & 0x1F) |           /* 5 lower bits from insn */
           (((int32_t)insn >> 20) & 0x1F);  /* 31:25 and 5 clear bits */
    unsigned memidx = 0;   /* mmu, always 0 in linux-user mode */

    TCGv va = imm ? temp_new_rsum(cpu_gpr[rs1], imm) : cpu_gpr[rs1];
    TCGv vs = cpu_fpr[rs2];

    switch(BITFIELD(insn, 14, 12)) {
        case /* 010 */ 2: tcg_gen_qemu_st32(vs, va, memidx); break;
        case /* 011 */ 3: tcg_gen_qemu_st64(vs, va, memidx); break;
        default: gen_illegal(dc);
    }

    if(imm) tcg_temp_free(va);
}

/* Fused multiply-add: rd = ±(rs1 x rs2 ± rs3); FMADD, FMSUB, FNMADD, FNMSUB.
   Lots of common code make it easier to funnel all four major opcodes here
   and do a big switch on size-op combo. */

static void gen_fmadd(DC, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned rm = BITFIELD(insn, 14, 12);
    unsigned rs1 = BITFIELD(insn, 19, 15);
    unsigned rs2 = BITFIELD(insn, 24, 20);
    unsigned rs3 = BITFIELD(insn, 31, 27);

    unsigned opcode =
            (BITFIELD(insn, 26, 25) << 2) | /* 00=float32, 01=float64 */
            (BITFIELD(insn, 3, 2));         /* 00=FMADD, ..., 11=FNMSUB */

    TCGv fd = cpu_fpr[rd];
    TCGv f1 = cpu_fpr[rs1];
    TCGv f2 = cpu_fpr[rs2];
    TCGv f3 = cpu_fpr[rs3];
    TCGv_ptr ep = cpu_env;
    TCGv_i32 vm = tcg_const_i32(rm);

    switch(opcode) {
        case /* 00.00 */ 0: gen_helper_fmadd_s(fd, ep, f1, f2, f3, vm); break;
        case /* 01.00 */ 4: gen_helper_fmadd_d(fd, ep, f1, f2, f3, vm); break;
        case /* 00.01 */ 1: gen_helper_fmsub_s(fd, ep, f1, f2, f3, vm); break;
        case /* 01.01 */ 5: gen_helper_fmsub_d(fd, ep, f1, f2, f3, vm); break;
        case /* 00.10 */ 2: gen_helper_fnmadd_s(fd, ep, f1, f2, f3, vm); break;
        case /* 01.10 */ 6: gen_helper_fnmadd_d(fd, ep, f1, f2, f3, vm); break;
        case /* 00.11 */ 3: gen_helper_fnmsub_s(fd, ep, f1, f2, f3, vm); break;
        case /* 01.11 */ 7: gen_helper_fnmsub_d(fd, ep, f1, f2, f3, vm); break;
        default: gen_illegal(dc);
    }

    tcg_temp_free_i32(vm);
}

/* FP sign-injection ops are simple bit ops and do not need softfloat;
   fw is 32 for S and 64 for D variants regardless of register width
   which is 64 in both cases for RV32D/RV64D.

   The code below depends on tl == i64 and TCGv == TCGv_i64. */

static void gen_fsgnj(DC, TCGv fd, TCGv f1, TCGv f2, int rm, int fw)
{
    TCGv sign = tcg_temp_new();
    TCGv base = tcg_temp_new();

    tcg_gen_andi_tl(sign, f2, 1 << (fw - 1));   /* 1<<31, sign bit */
    tcg_gen_andi_tl(base, f1, (1 << fw) - 1);   /* 1<<32 - 1 = 0xFFFFFFFF */
    tcg_gen_xor_tl(base, base, sign);           /* clear sign bit in base */

    switch(rm) {
        case /* 001 */ 1: tcg_gen_xori_tl(sign, sign, 1 << (fw - 1));
        case /* 000 */ 0: tcg_gen_or_tl(fd, base, sign); break;
        case /* 010 */ 2: tcg_gen_xor_tl(fd, base, sign); break;
        default: gen_illegal(dc);
    }

    tcg_temp_free(base);
    tcg_temp_free(sign);
}

static void gen_fminmax_s(DC, TCGv fd, TCGv_ptr ep, TCGv f1, TCGv f2, int rm)
{
    switch(rm) {
        case /* 000 */ 0: gen_helper_fmin_s(fd, ep, f1, f2); break;
        case /* 001 */ 1: gen_helper_fmax_s(fd, ep, f1, f2); break;
        default: gen_illegal(dc);
    }
}

static void gen_fminmax_d(DC, TCGv fd, TCGv_ptr ep, TCGv f1, TCGv f2, int rm)
{
    switch(rm) {
        case /* 000 */ 0: gen_helper_fmin_d(fd, ep, f1, f2); break;
        case /* 001 */ 1: gen_helper_fmax_d(fd, ep, f1, f2); break;
        default: gen_illegal(dc);
    }
}

static void gen_fcmp_s(DC, TCGv vd, TCGv_ptr ep, TCGv f1, TCGv f2, int rm)
{
    switch(rm) {
        case /* 000 */ 0: gen_helper_fle_s(vd, ep, f1, f2); break;
        case /* 001 */ 1: gen_helper_flt_s(vd, ep, f1, f2); break;
        case /* 010 */ 2: gen_helper_feq_s(vd, ep, f1, f2); break;
        default: gen_illegal(dc);
    }
}

static void gen_fcmp_d(DC, TCGv vd, TCGv_ptr ep, TCGv f1, TCGv f2, int rm)
{
    switch(rm) {
        case /* 000 */ 0: gen_helper_fle_d(vd, ep, f1, f2); break;
        case /* 001 */ 1: gen_helper_flt_d(vd, ep, f1, f2); break;
        case /* 010 */ 2: gen_helper_feq_d(vd, ep, f1, f2); break;
        default: gen_illegal(dc);
    }
}

/* Float-integer conversion. S/D variants differ in func value *and* use
   non-intersecting rs2 ranges. It would be possible to merge xs+xd and
   sx+dx if not for the need to signal invalid func:rs2 combos. */

static void gen_fcvt_xs(DC, TCGv vd, TCGv_ptr ep, TCGv f1, TCGv_i32 vm, int rs2)
{
    switch(rs2) {
        case /* 00000 */ 0: gen_helper_fcvt_w_s(vd, ep, f1, vm); break;
        case /* 00001 */ 1: gen_helper_fcvt_wu_s(vd, ep, f1, vm); break;
        default: gen_illegal(dc);
    }
}

static void gen_fcvt_xd(DC, TCGv vd, TCGv_ptr ep, TCGv f1, TCGv_i32 vm, int rs2)
{
    switch(rs2) {
        case /* 00010 */ 2: gen_helper_fcvt_l_d(vd, ep, f1, vm); break;
        case /* 00011 */ 3: gen_helper_fcvt_lu_d(vd, ep, f1, vm); break;
        default: gen_illegal(dc);
    }
}

static void gen_fcvt_sx(DC, TCGv fd, TCGv_ptr ep, TCGv v1, TCGv_i32 vm, int rs2)
{
    switch(rs2) {
        case /* 00000 */ 0: gen_helper_fcvt_s_w(fd, ep, v1, vm); break;
        case /* 00001 */ 1: gen_helper_fcvt_s_wu(fd, ep, v1, vm); break;
        default: gen_illegal(dc);
    }
}

static void gen_fcvt_dx(DC, TCGv fd, TCGv_ptr ep, TCGv v1, TCGv_i32 vm, int rs2)
{
    switch(rs2) {
        case /* 00010 */ 2: gen_helper_fcvt_d_l(fd, ep, v1, vm); break;
        case /* 00011 */ 3: gen_helper_fcvt_d_lu(fd, ep, v1, vm); break;
        default: gen_illegal(dc);
    }
}

/* FMV are just moves, but they share major opcodes with FCLASS. */

static void gen_fmv_xs(DC, TCGv vd, TCGv f1, unsigned rm)
{
    switch(rm) {
        case /* 000 */ 0: tcg_gen_ext32s_tl(vd, f1); break;
        case /* 001 */ 1: gen_helper_fclass_s(vd, f1); break;
        default: gen_illegal(dc);
    }
}

static void gen_fmv_xd(DC, TCGv vd, TCGv f1, unsigned rm)
{
    switch(rm) {
        case /* 000 */ 0: tcg_gen_mov_tl(vd, f1); break;
        case /* 001 */ 1: gen_helper_fclass_d(vd, f1); break;
        default: gen_illegal(dc);
    }
}

static void gen_fmv_sx(DC, TCGv fd, TCGv v1, unsigned rm)
{
    switch(rm) {
        case /* 000 */ 0: tcg_gen_ext32s_tl(fd, v1); break;
        default: gen_illegal(dc);
    }
}

static void gen_fmv_dx(DC, TCGv fd, TCGv v1, unsigned rm)
{
    switch(rm) {
        case /* 000 */ 0: tcg_gen_mov_tl(fd, v1); break;
        default: gen_illegal(dc);
    }
}

static void gen_opfp(DC, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned rm = BITFIELD(insn, 14, 12);
    unsigned rs1 = BITFIELD(insn, 19, 15);
    unsigned rs2 = BITFIELD(insn, 24, 20);
    unsigned func = BITFIELD(insn, 31, 25);

    TCGv_ptr ep = cpu_env;

    TCGv fd = cpu_fpr[rd];
    TCGv f1 = cpu_fpr[rs1];
    TCGv f2 = cpu_fpr[rs2];

    TCGv vd = cpu_gpr[rd];
    TCGv v1 = cpu_gpr[rs1];
    TCGv_i32 vm = tcg_const_i32(rm);
    int rs2d = (rs2 << 1) | (insn & 1);

    switch(func) {
        /* Single-precision */
        case /* 0000000 */ 0x00: gen_helper_fadd_s(fd, ep, f1, f2, vm); break;
        case /* 0000100 */ 0x04: gen_helper_fsub_s(fd, ep, f1, f2, vm); break;
        case /* 0001000 */ 0x08: gen_helper_fmul_s(fd, ep, f1, f2, vm); break;
        case /* 0001100 */ 0x0C: gen_helper_fdiv_s(fd, ep, f1, f2, vm); break;
        case /* 0101100 */ 0x2C: gen_helper_fsqrt_s(fd, ep, f2, vm); break;
        case /* 0010000 */ 0x10: gen_fsgnj(dc, fd, f1, f2, rm, 32); break;
        case /* 0010100 */ 0x14: gen_fminmax_s(dc, fd, ep, f1, f2, rm); break;
        case /* 1010000 */ 0x50: gen_fcmp_s(dc, fd, ep, f1, f2, rm); break;
        /* Double-precision */
        case /* 0000001 */ 0x01: gen_helper_fadd_d(fd, ep, f1, f2, vm); break;
        case /* 0000101 */ 0x05: gen_helper_fsub_d(fd, ep, f1, f2, vm); break;
        case /* 0001001 */ 0x09: gen_helper_fmul_d(fd, ep, f1, f2, vm); break;
        case /* 0001101 */ 0x0D: gen_helper_fdiv_d(fd, ep, f1, f2, vm); break;
        case /* 0101101 */ 0x2D: gen_helper_fsqrt_d(fd, ep, f2, vm); break;
        case /* 0010001 */ 0x11: gen_fsgnj(dc, fd, f1, f2, rm, 64); break;
        case /* 0010101 */ 0x15: gen_fminmax_d(dc, fd, ep, f1, f2, rm); break;
        case /* 1010001 */ 0x51: gen_fcmp_d(dc, fd, ep, f1, f2, rm); break;
        /* Float-integer conversion */
        case /* 1100000 */ 0x60: gen_fcvt_xs(dc, vd, ep, f1, vm, rs2d); break;
        case /* 1101000 */ 0x68: gen_fcvt_sx(dc, fd, ep, v1, vm, rs2d); break;
        case /* 1100001 */ 0x61: gen_fcvt_xd(dc, vd, ep, f1, vm, rs2d); break;
        case /* 1101001 */ 0x69: gen_fcvt_dx(dc, fd, ep, v1, vm, rs2d); break;
        /* Float-Integer moves (and fclassify) */
        case /* 1110000 */ 0x70: gen_fmv_xs(dc, vd, f1, rm); break;
        case /* 1111000 */ 0x78: gen_fmv_sx(dc, fd, v1, rm); break;
        case /* 1110001 */ 0x71: gen_fmv_xd(dc, vd, f1, rm); break;
        case /* 1111001 */ 0x79: gen_fmv_dx(dc, fd, v1, rm); break;
        default: gen_illegal(dc);
    }

    tcg_temp_free_i32(vm);
}

/* Instructions are decoded in two jumps: major opcode first,
   then whatever func* bits are used to determine the actual op.
   This makes encoding irregularities way easier to handle
   but requires each major opcode handler to gen EXCP_ILLEGAL
   on its own if necessary.

   Any path that generates no code and no exceptions results
   in a silent nop.

   Opcode values are "named" via function names next to them.
   Using symbolic constants instead of numeric opcode values
   does not add to readability, and makes checking against
   ISA listings way harder than it should be. */

static void decode(DC, uint32_t insn)
{
    switch(insn & 0x7F) {
        case /* 0110111 */ 0x37: gen_lui(dc, insn); break;
        case /* 0010111 */ 0x17: gen_auipc(dc, insn); break;
        case /* 1101111 */ 0x6F: gen_jal(dc, insn); break;
        case /* 1100111 */ 0x67: gen_jalr(dc, insn); break;
        case /* 1100011 */ 0x63: gen_branch(dc, insn); break;
        case /* 0000011 */ 0x03: gen_load(dc, insn); break;
        case /* 0100011 */ 0x23: gen_store(dc, insn); break;
        case /* 0010011 */ 0x13: gen_opimm(dc, insn); break;
        case /* 0011011 */ 0x1B: gen_opimm32(dc, insn); break;
        case /* 0110011 */ 0x33: gen_op(dc, insn); break;
        case /* 0111011 */ 0x3B: gen_op32(dc, insn); break;
        case /* 1110011 */ 0x73: gen_system(dc, insn); break;
        case /* 0001111 */ 0x0F: gen_miscmem(dc, insn); break;
        case /* 0000111 */ 0x07: gen_loadfp(dc, insn); break;
        case /* 0100111 */ 0x27: gen_storefp(dc, insn); break;
        case /* 1000011 */ 0x43: gen_fmadd(dc, insn); break;
        case /* 1000111 */ 0x47: gen_fmadd(dc, insn); break;
        case /* 1001011 */ 0x4B: gen_fmadd(dc, insn); break;
        case /* 1001111 */ 0x4F: gen_fmadd(dc, insn); break;
        case /* 1010011 */ 0x53: gen_opfp(dc, insn); break;
        default: gen_illegal(dc);
    }
}

void gen_intermediate_code(CPURISCVState *env, struct TranslationBlock *tb)
{
    RISCVCPU *cpu = riscv_env_get_cpu(env);
    struct DisasContext ctx, *dc = &ctx;
    int num_insns = 0;
    int max_insns = tblock_max_insns(tb);
    uint32_t startpc = tb->pc;
    uint32_t next_page_start = (startpc & TARGET_PAGE_MASK) + TARGET_PAGE_SIZE;

    dc->tb = tb;
    dc->pc = tb->pc;
    dc->jump = false;

    gen_tb_start(tb);

    do {
        tcg_gen_insn_start(dc->pc);

        /* Assuming 4-byte insns only for now */
        uint32_t insn = cpu_ldl_code(&cpu->env, dc->pc);
        decode(dc, insn);

        dc->pc += 4;

        if(dc->jump)
            break;
        if(dc->pc >= next_page_start)
            break;
    } while ((num_insns++ < max_insns) && !tcg_op_buf_full());

    if(!dc->jump) {
        tcg_gen_movi_tl(cpu_pc, dc->pc);
        tcg_gen_exit_tb(0);
    }

    gen_tb_end(tb, num_insns);

    tb->size = dc->pc - startpc;
    tb->icount = num_insns;
}
