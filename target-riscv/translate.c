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

typedef struct DisasContext {
    TranslationBlock *tb;
    target_ulong pc;
    bool jump;
} DisasContext;

static TCGv_env cpu_env;
static TCGv cpu_pc;
static TCGv cpu_gpr[32];
#include "exec/gen-icount.h"

#define BITFIELD(src, end, start) \
            (((src) >> start) & ((1 << (end - start + 1)) - 1))
const target_long rv_signbit = ((target_long)1 << (TARGET_LONG_BITS - 1));

/* Register x0 is a zero-sink, always reads as 0 and writes there are ignored.
   Within qemu, it's an allocated TCGv holding 0, used directly for any read
   access and guarded against writes.

   Most uses of x0 should be implemented with different tcg opcodes
   (i.e. "mov rd, rs" instead of "or rd, rs, x0") but until that is done having
   a real register keeps the code simple and uniform, if perhaps suboptimal. */

static const char* const riscv_regnames[] = {
     "x0",  "x1",  "x2",  "x3",  "x4",  "x5",  "x6",  "x7",
    " x8",  "x9", "x10", "x11", "x12", "x13", "x14", "x15",
    "x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23",
    "x24", "x25", "x26", "x27", "x28", "x29", "x30", "x31",
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
                        riscv_regnames[i]);
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

static void gen_exception(DisasContext *dc, unsigned int excp)
{
    TCGv_i32 tmp = tcg_const_i32(excp);
    tcg_gen_movi_tl(cpu_pc, dc->pc);
    gen_helper_exception(cpu_env, tmp);
    tcg_temp_free_i32(tmp);
}

static TCGv temp_new_rsum(unsigned rs, int32_t imm)
{
    TCGv vs = tcg_temp_new();
    tcg_gen_mov_tl(vs, cpu_gpr[rs]);
    tcg_gen_addi_tl(vs, vs, imm);
    return vs;
}

/* Load upper immediate: rd = (imm << 12)
   Since imm is 31:12 in insn, there is no need to shift it. */

static void rv_LUI(struct DisasContext* dc, uint32_t insn)
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

static void rv_AUIPC(struct DisasContext* dc, uint32_t insn)
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

static void rv_SLLI(struct DisasContext* dc, TCGv vd, TCGv vs,
        unsigned shamt, unsigned flags)
{
    if(flags)
        gen_exception(dc, EXCP_ILLEGAL);
    else
        tcg_gen_shli_tl(vd, vs, shamt);
}

static void rv_SRxI(struct DisasContext* dc, TCGv vd, TCGv vs,
        unsigned shamt, unsigned flags)
{
    if(flags & ~(1<<5))
        gen_exception(dc, EXCP_ILLEGAL);
    else if(flags & (1<<5))
        tcg_gen_sari_tl(vd, vs, shamt);
    else
        tcg_gen_shri_tl(vd, vs, shamt);
}

/* Arithmetics with immediate: rd = rs op imm;
   ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI.

   Writes to x0 are nops -- except if the flags are wrong, in which case
   exception must be raised regardless of where the results goes.
   Flags are checked late, so a temp is used instead of x0.
   TCG optimizer should remove it as dead code anyway. */

static void rv_OPIMM(struct DisasContext* dc, uint32_t insn)
{
    int32_t imm = ((int32_t)insn) >> 20;
    unsigned rs = BITFIELD(insn, 19, 15);
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned shamt = BITFIELD(insn, 24, 20);
    unsigned flags = BITFIELD(insn, 31, 25);

    TCGv vd = rd ? cpu_gpr[rd] : tcg_temp_new();
    TCGv vs = cpu_gpr[rs];

    switch(BITFIELD(insn, 14, 12))
    {
        case /* 000 */ 0: tcg_gen_addi_tl(vd, vs, imm); break;
        case /* 100 */ 4: tcg_gen_xori_tl(vd, vs, imm); break;
        case /* 110 */ 6: tcg_gen_ori_tl(vd, vs, imm); break;
        case /* 111 */ 7: tcg_gen_andi_tl(vd, vs, imm); break;
        case /* 010 */ 2: tcg_gen_setcondi_tl(TCG_COND_LT,  vd, vs, imm); break;
        case /* 011 */ 3: tcg_gen_setcondi_tl(TCG_COND_LTU, vd, vs, imm); break;
        case /* 001 */ 1: rv_SLLI(dc, vd, vs, shamt, flags); break;
        case /* 101 */ 5: rv_SRxI(dc, vd, vs, shamt, flags); break;
        default: gen_exception(dc, EXCP_ILLEGAL);
    }

    if(!rd) tcg_temp_free(vd);
}

/* Like OPIMM but with 32-bit words on RV64; ADDIW, SLLIW, SRLIW, SRAIW */

static void rv_OPIMM32(struct DisasContext* dc, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned rs = BITFIELD(insn, 19, 15);
    int32_t imm = ((int32_t)insn >> 20);      /* ADDIW only */
    unsigned shamt = BITFIELD(insn, 24, 20);  /* SLLIW, SRLIW, SRAIW */
    unsigned flags = BITFIELD(insn, 31, 25);  /* SLLIW, SRLIW, SRAIW */

    TCGv vd = rd ? cpu_gpr[rd] : tcg_temp_new();
    TCGv vs = cpu_gpr[rs];

    switch(BITFIELD(insn, 14, 12))
    {
        case /* 000 */ 0: tcg_gen_addi_tl(vd, vs, imm); break;
        case /* 001 */ 1: rv_SLLI(dc, vd, vs, shamt, flags); break;
        case /* 101 */ 5: rv_SRxI(dc, vd, vs, shamt, flags); break;
        default: gen_exception(dc, EXCP_ILLEGAL); goto out;
    }

    tcg_gen_ext32s_tl(vd, vd);

out:
    if(!rd) tcg_temp_free(vd);
}

/* Logical SLR and arithm SLA shift right: rd = rs1 << rs2
   RISC-V apparently *ignores* high bits in shift amount register,
   so (v >> 75) == (v >> (75 % 64)) == (v >> 11) */

static void rv_SRx(TCGv vd, TCGv vs1, TCGv vs2, int arithm)
{
    TCGv shamt = tcg_temp_new();
    tcg_gen_andi_tl(shamt, vs2, TARGET_LONG_BITS - 1);

    if(arithm)
        tcg_gen_sar_tl(vd, vs1, shamt);
    else
        tcg_gen_shr_tl(vd, vs1, shamt);

    tcg_temp_free(shamt);
}

static void rv_SLL(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv shamt = tcg_temp_new();
    tcg_gen_andi_tl(shamt, vs2, TARGET_LONG_BITS - 1);
    tcg_gen_shl_tl(vd, vs1, shamt);
    tcg_temp_free(shamt);
}

static void rv_MUL(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv sink = tcg_temp_new();
    tcg_gen_muls2_tl(vd, sink, vs1, vs2);
    tcg_temp_free(sink);
}

static void rv_MULH(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv sink = tcg_temp_new();
    tcg_gen_muls2_tl(sink, vd, vs1, vs2);
    tcg_temp_free(sink);
}

static void rv_MULHU(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv sink = tcg_temp_new();
    tcg_gen_mulu2_tl(sink, vd, vs1, vs2);
    tcg_temp_free(sink);
}

/* Signed x unsigned case, not implemented in TCG.
   Workaround: (-s * u) = -(s * u) = ~(s * u) + 1.
   The final addition is two-word long. */

static void rv_MULHSU(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGLabel* skip = gen_new_label();
    TCGLabel* done = gen_new_label();

    TCGv sign = tcg_temp_local_new();
    tcg_gen_andi_tl(sign, vs1, rv_signbit);
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

    /* negate resl with carry */
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
    tcg_gen_setcondi_tl(TCG_COND_EQ, c2, vs1, rv_signbit);
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

static void rv_DIV(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vs1, vs2, 0);
    gen_div_overcheck(done, vd, vs2, vs2, 0);
    tcg_gen_div_tl(vd, vs1, vs2);

    gen_set_label(done);
}

static void rv_DIVU(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vs1, vs2, 0);
    tcg_gen_divu_tl(vd, vs1, vs2);

    gen_set_label(done);
}

static void rv_REM(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vs1, vs2, 1);
    gen_div_overcheck(done, vd, vs2, vs2, 1);
    tcg_gen_rem_tl(vd, vs1, vs2);

    gen_set_label(done);
}

static void rv_REMU(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vs1, vs2, 1);
    tcg_gen_remu_tl(vd, vs1, vs2);

    gen_set_label(done);
}

/* Register arithmetics: rd = rs1 op rs2
   ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND.

   3-bit func field is not enough to encode all possible
   ops here, so two extra bits are spilled into the upper
   imm field of insn. */

static void rv_OP(struct DisasContext* dc, uint32_t insn)
{
    unsigned flags = BITFIELD(insn, 31, 25);
    unsigned rs2 = BITFIELD(insn, 24, 20);
    unsigned rs1 = BITFIELD(insn, 19, 15);
    unsigned func = BITFIELD(insn, 14, 12); /* 3 bits initially */
    unsigned rd = BITFIELD(insn, 11, 7);

    if(flags & ~((1<<5) | (1<<0)))
        func = -1;        /* force default: case below */
    if(flags & (1<<0))
        func |= (1<<3);   /* prefix func with mul bit */
    if(flags & (1<<5))
        func |= (1<<4);   /* prefix func with arithm bit */

    TCGv vd = rd ? cpu_gpr[rd] : tcg_temp_local_new();
    TCGv vs1 = cpu_gpr[rs1];
    TCGv vs2 = cpu_gpr[rs2];

    switch(func)
    {
        case /* 00.000 */ 0: tcg_gen_add_tl(vd, vs1, vs2); break;
        case /* 10.000 */16: tcg_gen_sub_tl(vd, vs1, vs2); break;
        case /* 00.100 */ 4: tcg_gen_xor_tl(vd, vs1, vs2); break;
        case /* 00.110 */ 6: tcg_gen_or_tl(vd, vs1, vs2); break;
        case /* 00.111 */ 7: tcg_gen_and_tl(vd, vs1, vs2); break;
        case /* 00.010 */ 2: tcg_gen_setcond_tl(TCG_COND_LT, vd, vs1, vs2); break;
        case /* 00.011 */ 3: tcg_gen_setcond_tl(TCG_COND_LTU, vd, vs1, vs2); break;
        case /* 00.001 */ 1: rv_SLL(vd, vs1, vs2); break;
        case /* 00.101 */ 5: rv_SRx(vd, vs1, vs2, 0); break;
        case /* 10.101 */21: rv_SRx(vd, vs1, vs2, 1); break;
        case /* 01.000 */ 8: rv_MUL(vd, vs1, vs2); break;
        case /* 01.001 */ 9: rv_MULH(vd, vs1, vs2); break;
        case /* 01.010 */10: rv_MULHSU(vd, vs1, vs2); break;
        case /* 01.011 */11: rv_MULHU(vd, vs1, vs2); break;
        case /* 01.100 */12: rv_DIV(vd, vs1, vs2); break;
        case /* 01.101 */13: rv_DIVU(vd, vs1, vs2); break;
        case /* 01.110 */14: rv_REM(vd, vs1, vs2); break;
        case /* 01.111 */15: rv_REMU(vd, vs1, vs2); break;
        default: gen_exception(dc, EXCP_ILLEGAL);
    }

    if(!rd) tcg_temp_free(vd);
}

/* Like OP but with 32-bit words on RV64. Same opcode encoding,
   but some regular opcodes have no *W equivalent so just calling
   rv_OP here is not an option. */

static void rv_OP32(struct DisasContext* dc, uint32_t insn)
{
    unsigned flags = BITFIELD(insn, 31, 25);
    unsigned rs2 = BITFIELD(insn, 24, 20);
    unsigned rs1 = BITFIELD(insn, 19, 15);
    unsigned func = BITFIELD(insn, 14, 12); /* 3 bits initially */
    unsigned rd = BITFIELD(insn, 11, 7);

    if(flags & ~((1<<5) | (1<<0)))
        func = -1;        /* force default: case below */
    if(flags & (1<<0))
        func |= (1<<3);   /* prefix func with mul bit */
    if(flags & (1<<5))
        func |= (1<<4);   /* prefix func with arithm bit */

    TCGv vd = rd ? cpu_gpr[rd] : tcg_temp_local_new();
    TCGv vs1 = cpu_gpr[rs1];
    TCGv vs2 = cpu_gpr[rs2];

    /* cast rs1 to 32 bit, using rd as a temp register */
    tcg_gen_ext32s_tl(vd, vs1);

    switch(func)
    {
        case /* 00.000 */ 0: tcg_gen_add_tl(vd, vd, vs2); break;
        case /* 10.000 */16: tcg_gen_sub_tl(vd, vd, vs2); break;
        case /* 00.001 */ 1: rv_SLL(vd, vd, vs2); break;
        case /* 00.101 */ 5: rv_SRx(vd, vd, vs2, 0); break;
        case /* 10.101 */21: rv_SRx(vd, vd, vs2, 1); break;
        default: gen_exception(dc, EXCP_ILLEGAL); goto out;
    }

    /* sign-extend 32-bit result to fill the whole rd */
    tcg_gen_ext32s_tl(vd, vd);

out:
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

static void rv_JAL(struct DisasContext* dc, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    uint32_t imm =
            (BITFIELD(insn, 31, 31) << 20) |
            (BITFIELD(insn, 19, 12) << 12) |
            (BITFIELD(insn, 20, 20) << 11) |
            (BITFIELD(insn, 30, 21) << 1);

    if(imm % 3) {
        gen_exception(dc, EXCP_ILLEGAL);
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

static void rv_JALR(struct DisasContext* dc, uint32_t insn)
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
    gen_exception(dc, EXCP_ILLEGAL); /* should be misalignment */

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

static void rv_BRANCH(struct DisasContext* dc, uint32_t insn)
{
    unsigned rs1 = BITFIELD(insn, 19, 15);
    unsigned rs2 = BITFIELD(insn, 24, 20);
    int16_t imm =
            (BITFIELD(insn, 11,  8) <<  1) |        /*  4:1 */
            (BITFIELD(insn, 30, 25) <<  5) |        /* 10:5 */
            (BITFIELD(insn,  7,  7) << 11) |        /* 11 */
            (((((int32_t)insn) >> 30) & 1) << 12);  /* 12, sign bit */

    if(imm % 3) {
        gen_exception(dc, EXCP_ILLEGAL);
        return;
    }

    TCGv vs1 = cpu_gpr[rs1];
    TCGv vs2 = cpu_gpr[rs2];
    TCGLabel* l = gen_new_label();

    switch(BITFIELD(insn, 14, 12))
    {
        case /* 000 */ 0: tcg_gen_brcond_tl(TCG_COND_NE, vs1, vs2, l); break;
        case /* 001 */ 1: tcg_gen_brcond_tl(TCG_COND_EQ, vs1, vs2, l); break;
        case /* 100 */ 4: tcg_gen_brcond_tl(TCG_COND_GE, vs1, vs2, l); break;
        case /* 101 */ 5: tcg_gen_brcond_tl(TCG_COND_LT, vs1, vs2, l); break;
        case /* 110 */ 6: tcg_gen_brcond_tl(TCG_COND_GEU, vs1, vs2, l); break;
        case /* 111 */ 7: tcg_gen_brcond_tl(TCG_COND_LTU, vs1, vs2, l); break;
        default: gen_exception(dc, EXCP_ILLEGAL); return;
    }

    tcg_gen_movi_tl(cpu_pc, dc->pc + imm);
    tcg_gen_exit_tb(0);

    gen_set_label(l);
}

/* Memory load: rd = [rs + imm]; LB, LH, LW, LD, LBU, LHU */

static void rv_LOAD(struct DisasContext* dc, uint32_t insn)
{
    int32_t imm = ((int32_t)insn) >> 20;
    unsigned rs = BITFIELD(insn, 19, 15);
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned memidx = 0;   /* mmu, always 0 in linux-user mode */

    TCGv va = imm ? temp_new_rsum(rs, imm) : cpu_gpr[rs];
    TCGv vd = rd ? cpu_gpr[rd] : tcg_temp_new();

    switch(BITFIELD(insn, 14, 12))
    {
        case /* 000 */ 0: tcg_gen_qemu_ld8s(vd, va, memidx); break;
        case /* 001 */ 1: tcg_gen_qemu_ld16s(vd, va, memidx); break;
        case /* 010 */ 2: tcg_gen_qemu_ld32s(vd, va, memidx); break;
        case /* 100 */ 4: tcg_gen_qemu_ld8u(vd, va, memidx); break;
        case /* 101 */ 5: tcg_gen_qemu_ld16u(vd, va, memidx); break;
        case /* 110 */ 6: tcg_gen_qemu_ld32u(vd, va, memidx); break;
        default: gen_exception(dc, EXCP_ILLEGAL);
    }

    if(!rd) tcg_temp_free(vd);
    if(imm) tcg_temp_free(va);
}

/* Memory store: [rs1 + imm] = rs2; SB, SH, SW, SD. */

static void rv_STORE(struct DisasContext* dc, uint32_t insn)
{
    unsigned rs1 = BITFIELD(insn, 19, 15);  /* address */
    unsigned rs2 = BITFIELD(insn, 24, 20);  /* data to store */
    int32_t imm = (insn & 0x1F) |           /* 5 lower bits from insn */
           (((int32_t)insn >> 20) & 0x1F);  /* 31:25 and 5 clear bits */
    unsigned memidx = 0;   /* mmu, always 0 in linux-user mode */

    TCGv va = imm ? temp_new_rsum(rs1, imm) : cpu_gpr[rs1];
    TCGv vs = cpu_gpr[rs2];

    switch(BITFIELD(insn, 14, 12))
    {
        case /* 000 */ 0: tcg_gen_qemu_st8(vs, va, memidx); break;
        case /* 001 */ 1: tcg_gen_qemu_st16(vs, va, memidx); break;
        case /* 010 */ 2: tcg_gen_qemu_st32(vs, va, memidx); break;
        case /* 011 */ 3: tcg_gen_qemu_st64(vs, va, memidx); break;
        default: gen_exception(dc, EXCP_ILLEGAL);
    }

    if(imm) tcg_temp_free(va);
}

/* Memory FENCE and FENCE.I with i-cache flush.
   No-ops for now; rd, rs and imm ignored as per spec. */

static void rv_MISCMEM(struct DisasContext* dc, uint32_t insn)
{
    switch(BITFIELD(insn, 14, 12))
    {
        case 0: /* FENCE */
        case 1: /* FENCE.I */
            break;
        default: gen_exception(dc, EXCP_ILLEGAL);
    }
}

/* Privileged insns, plus ECALL and EBREAK which may be used in U-level. */

static void rv_PRIV(struct DisasContext* dc, uint32_t insn)
{
    switch(BITFIELD(insn, 31, 20)) {
        case 0: gen_exception(dc, EXCP_SYSCALL);
        case 1: gen_exception(dc, EXCP_ILLEGAL); /* EBREAK, not implemented */
        default: gen_exception(dc, EXCP_ILLEGAL);
    }
}

/* Syscalls and CSR ops. These are actuall two completely unrelated
   instructions sharing a single opcode. */

static void rv_SYSTEM(struct DisasContext* dc, uint32_t insn)
{
    unsigned func = BITFIELD(insn, 14, 12);

    switch(func) {
        case /* 000 */ 0: rv_PRIV(dc, insn); return;
        case /* 100 */ 4: gen_exception(dc, EXCP_ILLEGAL); return;
        /* all other possible values are CRS-related */
        default: gen_exception(dc, EXCP_ILLEGAL); /* not supported atm  */
    }
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

static void decode(struct DisasContext* dc, uint32_t insn)
{
    switch(insn & 0x7F)
    {
        case /* 0110111 */ 0x37: rv_LUI(dc, insn); break;
        case /* 0010111 */ 0x17: rv_AUIPC(dc, insn); break;
        case /* 1101111 */ 0x6F: rv_JAL(dc, insn); break;
        case /* 1100111 */ 0x67: rv_JALR(dc, insn); break;
        case /* 1100011 */ 0x63: rv_BRANCH(dc, insn); break;
        case /* 0000011 */ 0x03: rv_LOAD(dc, insn); break;
        case /* 0100011 */ 0x23: rv_STORE(dc, insn); break;
        case /* 0010011 */ 0x13: rv_OPIMM(dc, insn); break;
        case /* 0011011 */ 0x1B: rv_OPIMM32(dc, insn); break;
        case /* 0110011 */ 0x33: rv_OP(dc, insn); break;
        case /* 0111011 */ 0x3B: rv_OP32(dc, insn); break;
        case /* 1110011 */ 0x73: rv_SYSTEM(dc, insn); break;
        case /* 0001111 */ 0x0F: rv_MISCMEM(dc, insn); break;
        default: gen_exception(dc, EXCP_ILLEGAL);
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
