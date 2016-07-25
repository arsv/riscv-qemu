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
    uint32_t is_jmp;
} DisasContext;

static TCGv_env cpu_env;
static TCGv cpu_pc;
static TCGv cpu_gpr[32];
#include "exec/gen-icount.h"

#define BITFIELD(src, end, start) \
            (((src) >> start) & ((1 << (end - start + 1)) - 1))

void restore_state_to_opc(CPURISCVState *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->pc = data[0];
}

static void gen_exception(DisasContext *dc, unsigned int excp)
{
    TCGv_i32 tmp = tcg_const_i32(excp);
    tcg_gen_movi_tl(cpu_pc, dc->pc);
    gen_helper_exception(cpu_env, tmp);
    tcg_temp_free_i32(tmp);
}

static int tblock_max_insns(struct TranslationBlock *tb)
{
    int max = tb->cflags & CF_COUNT_MASK;

    if(!max)
        max = CF_COUNT_MASK;
    if(max > TCG_MAX_INSNS)
        max = TCG_MAX_INSNS;

    return max;
}

static void rv_LUI(struct DisasContext* dc, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    uint32_t uimm = insn & 0xFFFFF000;
    TCGv vd = cpu_gpr[rd];

    if(!rd) return; /* write to x0 is nop */

    tcg_gen_movi_tl(vd, uimm);
    tcg_gen_ext32s_tl(vd, vd);
}

static void rv_AUIPC(struct DisasContext* dc, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    uint32_t uimm = insn & 0xFFFFF000;
    TCGv vd = cpu_gpr[rd];

    if(!rd) return; /* write to x0 is nop */

    tcg_gen_movi_tl(vd, uimm);
    tcg_gen_ext32s_tl(vd, vd);
    /* cpu_pc is not updated on each tick and may be off */
    tcg_gen_add_tl(vd, vd, tcg_const_tl(dc->pc));
}

static void rv_ADDI(struct DisasContext* dc, int rd, int rs, int32_t imm)
{
    TCGv vd = cpu_gpr[rd];
    TCGv vs = cpu_gpr[rs];

    tcg_gen_addi_tl(vd, vs, imm);
}

static void rv_OPIMM(struct DisasContext* dc, uint32_t insn)
{
    int32_t imm = ((int32_t)insn) >> 20;
    unsigned rs = BITFIELD(insn, 19, 15);
    unsigned rd = BITFIELD(insn, 11, 7);

    if(!rd) return; /* write to x0 is nop */

    switch(BITFIELD(insn, 14, 12)) {
        case 0x0: rv_ADDI(dc, rd, rs, imm); return;
        default: gen_exception(dc, EXCP_ILLEGAL); return;
    }
}

static void rv_ECALL(struct DisasContext* dc, uint32_t imm)
{
    if(imm & 1)
        gen_exception(dc, EXCP_ILLEGAL); /* EBREAK, not implemented */
    else
        gen_exception(dc, EXCP_SYSCALL); /* ECALL proper */
}

static void rv_SYSTEM(struct DisasContext* dc, uint32_t insn)
{
    uint32_t imm = BITFIELD(insn, 31, 20);

    switch(BITFIELD(insn, 14, 12)) {
        case 0x0: rv_ECALL(dc, imm); return;
        default: gen_exception(dc, EXCP_ILLEGAL); return;
    }
}

/* Instructions are decoded in two jumps: first major opcode,
   then whatever func* bits are used to determine the actual op.
   This makes encoding irregularities way easier to handle
   but requires each major opcode handler to gen EXCP_ILLEGAL
   on its own if necessary. A path that generates no code and
   no exceptions results in a silent nop.

   Opcode values are "named" via function names next to them.
   Using symbolic constants instead of numeric opcode values
   does not add to readability, and makes checking against
   intruction set listings way harder than it should be. */

static void decode(struct DisasContext* dc, uint32_t insn)
{
    switch(insn & 0x7F)
    {
        case /* 0110111 */ 0x37: rv_LUI(dc, insn);    return;
        case /* 0010111 */ 0x17: rv_AUIPC(dc, insn);  return;
        //case /* 1101111 */ 0x6F: rv_JAL(dc, insn);    return;
        //case /* 1100111 */ 0x67: rv_JALR(dc, insn);   return;
        //case /* 1100011 */ 0x63: rv_BRANCH(dc, insn); return;
        //case /* 0000011 */ 0x03: rv_LOAD(dc, insn);   return;
        //case /* 0100011 */ 0x23: rv_STORE(dc, insn);  return;
        case /* 0010011 */ 0x13: rv_OPIMM(dc, insn);  return;
        //case /* 0110011 */ 0x33: rv_OP(dc, insn);     return;
        case /* 1110011 */ 0x73: rv_SYSTEM(dc, insn); return;
        //default: printf("%8lX: %08X op=0x%02X\n", dc->pc, insn, insn & 0x7F);
    }
    gen_exception(dc, EXCP_ILLEGAL);
    dc->is_jmp = DISAS_UPDATE;
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
    dc->is_jmp = DISAS_NEXT;

    gen_tb_start(tb);

    do {
        tcg_gen_insn_start(dc->pc);

        /* Assuming 4-byte insns only for now */
        uint32_t insn = cpu_ldl_code(&cpu->env, dc->pc);
        decode(dc, insn);

        num_insns++;
        dc->pc += 4;
    } while (dc->is_jmp == DISAS_NEXT
             && !tcg_op_buf_full()
             && (dc->pc < next_page_start)
             && num_insns < max_insns);

    switch (dc->is_jmp) {
        case DISAS_NEXT:
            tcg_gen_movi_tl(cpu_pc, dc->pc);
        case DISAS_UPDATE:
            tcg_gen_exit_tb(0);
            break;
        case DISAS_JUMP:
        case DISAS_TB_JUMP:
            break;
    }

    gen_tb_end(tb, num_insns);

    tb->size = dc->pc - startpc;
    tb->icount = num_insns;
}

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
