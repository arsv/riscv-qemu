/*
 * RISC-V translation
 *
 * Copyright (c) 2016 Alex Suykov <alex.suykov@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

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

/* In this file, chunks of RISC-V code are translated into native code
   to be run on the host CPU. See
       http://wiki.qemu.org/Documentation/TCG
   on how it happens, and User-Level ISA Specification v2.1 from
       https://riscv.org/specifications/
   on what is being translated. Chapter 9 Instruction Set Listings
   is where most of the raw binary opcodes come from. */

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
    target_ulong pc;  /* current pc, the insn being decoded */
    target_ulong npc; /* next pc, skips instruction being decoded */
    bool jump;
    bool singlestep;
} DisasContext;

#define DC DisasContext* dc

/* TCG references to CPU registers */

typedef TCGv_i64 TCGf;

static TCGv_env cpu_env;
static TCGv cpu_pc;
static TCGv cpu_gpr[32];
static TCGf cpu_fpr[32];
static TCGv_i32 cpu_amoinsn;

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

/* Arguments for gen_exit_tb(), see comments there */
#define EXIT_TB_DOWN 0
#define EXIT_TB_SIDE 1
#define ADDRESS_UNKNOWN 0

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
        cpu_fpr[i] = tcg_global_mem_new_i64(cpu_env,
                        offsetof(CPURISCVState, fpr[i]),
                        riscv_fprnames[i]);

    cpu_amoinsn = tcg_global_mem_new_i32(cpu_env,
                    offsetof(CPURISCVState, amoinsn),
                    "amoinsn");
}

void restore_state_to_opc(CPURISCVState *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->pc = data[0];
}

/* All the stuff below this line is gen_intermediate_code()

   The code that used to be here has been split into several files
   around the time translate.c hit 50KB mark. My envy for those still
   capable of thinking straight while working with 400+ KB monsters
   right there at ../target-arm knows no limits.

   Including .c files is a bad style but in this case the benefits
   of static functions outweight style concerns. All that is a single 
   huge function-module with a single entry point. */

#define IN_TRANSLATE_C
#include "translate_util.c"
#include "translate_rva.c"
#include "translate_rvf.c"
#include "translate_rvm.c"
#include "translate_rvi.c"
#include "translate_rvc.c"

/* Most insns are decoded in two jumps: major opcode first,
   then whatever func* bits are used to determine the actual op.
   This makes encoding irregularities way easier to handle
   but requires each major opcode handler to gen EXCP_ILLEGAL
   on its own if necessary. Any path that generates no code
   and no exceptions results in a silent nop.

   Opcode values are "named" via function names next to them.
   Using symbolic constants instead of numeric opcode values
   does not add to readability, and makes checking against
   ISA listings way harder than it should be. */

static void gen_one_4byte_insn(DC, uint32_t insn)
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
        case /* 0110011 */ 0x33: gen_op(dc, insn); break;
        case /* 1110011 */ 0x73: gen_system(dc, insn); break;
        case /* 0001111 */ 0x0F: gen_miscmem(dc, insn); break;
        case /* 0101111 */ 0x2F: gen_amo(dc, insn); break;
        case /* 0000111 */ 0x07: gen_loadfp(dc, insn); break;
        case /* 0100111 */ 0x27: gen_storefp(dc, insn); break;
        case /* 1000011 */ 0x43: gen_fmadd(dc, insn); break;
        case /* 1000111 */ 0x47: gen_fmadd(dc, insn); break;
        case /* 1001011 */ 0x4B: gen_fmadd(dc, insn); break;
        case /* 1001111 */ 0x4F: gen_fmadd(dc, insn); break;
        case /* 1010011 */ 0x53: gen_opfp(dc, insn); break;
#ifndef TARGET_RISCV32
        case /* 0011011 */ 0x1B: gen_opimm32(dc, insn); break;
        case /* 0111011 */ 0x3B: gen_op32(dc, insn); break;
#endif
        default: gen_illegal(dc);
    }
}

/* 4 byte insns are presumed to be much more frequent and the arch is
   little-endian, so it makes a lot of sense to go for an optimistic
   4-byte fetch. However, there's a nasty corner case with a 2-byte
   insn located at the end of the last accessible page. That particular
   very rare case must be handled carefuly to avoid illegal reads. */

static void fetch_gen_one_insn(DC, CPURISCVState *env)
{
    uint32_t insn;

    if(unlikely((dc->pc & ~TARGET_PAGE_MASK) == (~TARGET_PAGE_MASK - 1))) {
        insn = cpu_lduw_code(env, dc->pc);
        if((insn & 3) != 3)
            goto twobyte;
    }

    insn = cpu_ldl_code(env, dc->pc);

    if((insn & 3) != 3)
        goto twobyte;

    dc->npc = dc->pc + 4;
    gen_one_4byte_insn(dc, insn);
    dc->pc = dc->npc;
    return;

twobyte:
    insn &= 0xFFFF;
    dc->npc = dc->pc + 2;
    gen_one_2byte_insn(dc, insn);
    dc->pc = dc->npc;
    return;
}

static int tblock_max_insns(struct TranslationBlock *tb)
{
    if(singlestep) /* global -singlestep option; no relation to gdb */
        return 1;

    int max = tb->cflags & CF_COUNT_MASK;

    if(!max)
        max = CF_COUNT_MASK;
    if(max > TCG_MAX_INSNS)
        max = TCG_MAX_INSNS;

    return max;
}

void gen_intermediate_code(CPURISCVState *env, struct TranslationBlock *tb)
{
    RISCVCPU *cpu = riscv_env_get_cpu(env);
    CPUState *cs = CPU(cpu);

    struct DisasContext ctx, *dc = &ctx;
    int num_insns = 0;
    int max_insns = tblock_max_insns(tb);
    uint32_t startpc = tb->pc;
    uint32_t next_page_start = (startpc & TARGET_PAGE_MASK) + TARGET_PAGE_SIZE;

    dc->tb = tb;
    dc->pc = tb->pc;
    dc->jump = false;
    dc->singlestep = cs->singlestep_enabled;

    gen_tb_start(tb);

    do {
        tcg_gen_insn_start(dc->pc);

        if(unlikely(cpu_breakpoint_test(cs, dc->pc, BP_ANY))) {
            gen_excp_exit(dc, EXCP_DEBUG);
            break;
        }

        fetch_gen_one_insn(dc, &cpu->env);

        if(dc->jump || dc->singlestep)
            break;
        if(dc->pc >= next_page_start)
            break;
    } while ((num_insns++ < max_insns) && !tcg_op_buf_full());

    if(!dc->jump)
        gen_exit_tb(dc, EXIT_TB_DOWN, dc->pc);

    gen_tb_end(tb, num_insns);

    tb->size = dc->pc - startpc;
    tb->icount = num_insns;
}
