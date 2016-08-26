/* Subset I: basic integer ops and flow control */

#ifndef IN_TRANSLATE_C
#error this file is a part of translate.c, do not compile separately
#endif

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
    if(flags & ~(1<<4))
        gen_illegal(dc);
    else if(flags & (1<<4))
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
    unsigned shamt = BITFIELD(insn, 25, 20);
    unsigned flags = BITFIELD(insn, 31, 26);

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

#ifndef TARGET_RISCV32

static void gen_srxiw(DC, TCGv vd, TCGv vs, unsigned shamt, unsigned flags)
{
    if(flags & ~(1<<4)) {
        gen_illegal(dc);
    } else if(flags & (1<<4)) {
        tcg_gen_ext32s_tl(vd, vs);
        tcg_gen_sari_tl(vd, vd, shamt);
    } else {
        tcg_gen_ext32u_tl(vd, vs);
        tcg_gen_shri_tl(vd, vd, shamt);
    }
}

/* Like OPIMM but with 32-bit words on RV64; ADDIW, SLLIW, SRLIW, SRAIW */

static void gen_opimm32(DC, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned rs = BITFIELD(insn, 19, 15);
    int32_t imm = ((int32_t)insn >> 20);      /* ADDIW only */
    unsigned shamt = BITFIELD(insn, 25, 20);  /* SLLIW, SRLIW, SRAIW */
    unsigned flags = BITFIELD(insn, 31, 26);  /* SLLIW, SRLIW, SRAIW */

    TCGv vd = rd ? cpu_gpr[rd] : tcg_temp_new();
    TCGv vs = cpu_gpr[rs];

    switch(BITFIELD(insn, 14, 12)) {
        case /* 000 */ 0: tcg_gen_addi_tl(vd, vs, imm); break;
        case /* 001 */ 1: gen_slli(dc, vd, vs, shamt, flags); break;
        case /* 101 */ 5: gen_srxiw(dc, vd, vs, shamt, flags); break;
        default: gen_illegal(dc); goto out;
    }

    tcg_gen_ext32s_tl(vd, vd);

out:
    if(!rd) tcg_temp_free(vd);
}

#endif

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

/* Register arithmetics: rd = rs1 op rs2 */

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

#ifndef TARGET_RISCV32

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

#endif

/* Like OP but with 32-bit words on RV64.

   Register handling is not uniform here, some insns sign-extend,
   some zero-extend and some do not care at all. So each insn does
   it in its own function. */

#ifndef TARGET_RISCV32

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
        case /* 00.001 */ 1: gen_sllw(vd, vs1, vs2); break;
        case /* 00.101 */ 5: gen_srlw(vd, vs1, vs2); break;
        case /* 10.101 */21: gen_sraw(vd, vs1, vs2); break;
        case /* 01.000 */ 8: gen_mulw(vd, vs1, vs2); break;
        case /* 01.100 */12: gen_divw(vd, vs1, vs2); break;
        case /* 01.101 */13: gen_divuw(vd, vs1, vs2); break;
        case /* 01.110 */14: gen_remw(vd, vs1, vs2); break;
        case /* 01.111 */15: gen_remuw(vd, vs1, vs2); break;
        default: gen_illegal(dc);
    }

    if(!rd) tcg_temp_free(vd);
}

#endif

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
   Bit 1 is allowed to be non-zero now with two-byte opcodes implemented,
   so JAL never generates any exceptions at all. */

static void gen_jal(DC, uint32_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    /* insn   31   30:21   20   19:12   */
    /*  imm [ 20 | 10:1  | 11 | 19:12 ] */
    int32_t imm =
            (BITFIELD(insn, 19, 12) << 12) |
            (BITFIELD(insn, 20, 20) << 11) |
            (BITFIELD(insn, 30, 21) << 1);

    if(BITFIELD(insn, 31, 31)) /* sign bit */
        imm |= (-1 << 20);

    if(rd) /* JAL x0 is a plain jump */
        tcg_gen_movi_tl(cpu_gpr[rd], dc->npc);

    gen_exit_tb(dc, EXIT_TB_DOWN, dc->pc + imm);
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
        tcg_gen_movi_tl(cpu_gpr[rd], dc->npc);

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
    tcg_gen_mov_tl(cpu_pc, va);
    gen_exit_tb(dc, EXIT_TB_DOWN, ADDRESS_UNKNOWN);

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
            (BITFIELD(insn,  7,  7) << 11);         /* 11 */

    if(insn & (1<<31))
        imm |= (-1 << 12);                          /* 12, sign bit */

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

    gen_exit_tb(dc, EXIT_TB_SIDE, dc->pc + imm);

    gen_set_label(l);
    gen_exit_tb(dc, EXIT_TB_DOWN, dc->npc);
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
        case /* 100 */ 4: tcg_gen_qemu_ld8u(vd, va, memidx); break;
        case /* 101 */ 5: tcg_gen_qemu_ld16u(vd, va, memidx); break;
        case /* 110 */ 6: tcg_gen_qemu_ld32u(vd, va, memidx); break;
#ifndef TARGET_RISCV32
        case /* 011 */ 3: tcg_gen_qemu_ld64(vd, va, memidx); break;
#endif
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
    int32_t imm = BITFIELD(insn, 11, 7) |   /* 5 lower bits */
           (((int32_t)insn >> 20) & ~0x1F); /* 31:25 w/ 5 clear bits */
    unsigned memidx = 0;   /* mmu, always 0 in linux-user mode */

    TCGv va = imm ? temp_new_rsum(cpu_gpr[rs1], imm) : cpu_gpr[rs1];
    TCGv vs = cpu_gpr[rs2];

    switch(BITFIELD(insn, 14, 12)) {
        case /* 000 */ 0: tcg_gen_qemu_st8(vs, va, memidx); break;
        case /* 001 */ 1: tcg_gen_qemu_st16(vs, va, memidx); break;
        case /* 010 */ 2: tcg_gen_qemu_st32(vs, va, memidx); break;
#ifndef TARGET_RISCV32
        case /* 011 */ 3: tcg_gen_qemu_st64(vs, va, memidx); break;
#endif
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

static void gen_ecall(DC)
{
    gen_excp_exit(dc, EXCP_SYSCALL);
}

static void gen_priv(DC, uint32_t insn)
{
    switch(BITFIELD(insn, 31, 20)) {
        case 0: gen_ecall(dc); break;
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
