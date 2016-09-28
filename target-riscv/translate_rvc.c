/* Subset C: compressed (half-word, 16bit) instructions */

#ifndef IN_TRANSLATE_C
#error this file is a part of translate.c, do not compile separately
#endif

#define ONEBIT(insn, i) (((insn) >> i) & 1)

static int rv_ci_imm(uint16_t insn)
{
    int imm = BITFIELD(insn, 6, 2);

    if(BITFIELD(insn, 12, 12)) /* sign bit */
        imm |= -1 << 5;

    return imm;
}

static int rv_cj_imm(uint16_t insn)
{
    /* offset[ 11 | 4 | 9:8 | 10 | 6 | 7 | 3:1 | 5 ] */
    /*  insn   12  11  10:9    8   7   6   543   2   */
    int imm =
            (ONEBIT(insn, 11) <<  4) |
            (ONEBIT(insn, 10) <<  9) |
            (ONEBIT(insn,  9) <<  8) |
            (ONEBIT(insn,  8) << 10) |
            (ONEBIT(insn,  7) <<  6) |
            (ONEBIT(insn,  6) <<  7) |
            (BITFIELD(insn,  5, 3) << 1) |
            (ONEBIT(insn,  2) <<  5);

    if(ONEBIT(insn, 12))
        imm |= -1 << 11;

    return imm;
}

static int rv_cb_imm(uint16_t insn)
{
    /* offset[ 8 | 4:3 ]  [ 7:6 | 2:1 | 5 ] */
    /*  insn  12  11 10     6 5   4 3   2   */
    int imm =
            (ONEBIT(insn, 11) << 4) |
            (ONEBIT(insn, 10) << 3) |
            (ONEBIT(insn,  6) << 7) |
            (ONEBIT(insn,  5) << 6) |
            (ONEBIT(insn,  4) << 2) |
            (ONEBIT(insn,  3) << 1) |
            (ONEBIT(insn,  2) << 5);

    if(ONEBIT(insn, 12))
        imm |= -1 << 8;

    return imm;
}

/* Add immediate to sp, sp += imm*4. With rd=0, this is all-zero op
   which must be signalled as illegal. */

static void gen_addi4spn(DC, uint16_t insn)
{
    unsigned rd = 8 + BITFIELD(insn, 4, 2);
    unsigned imm =
            (ONEBIT(insn, 5) << 3) |
            (ONEBIT(insn, 6) << 2) |
            (BITFIELD(insn, 10, 7) << 6) |
            (BITFIELD(insn, 12, 11) << 4);

    if(!rd)
        gen_illegal(dc);
    else
        tcg_gen_addi_tl(cpu_gpr[rd], cpu_gpr[xSP], imm);
}

/* Load immediate */

static void gen_cli(DC, uint16_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    int imm = rv_ci_imm(insn);

    if(!rd) return;

    tcg_gen_movi_tl(cpu_gpr[rd], imm);
}

static void gen_caddi16sp(DC, uint16_t insn)
{
    TCGv vd = cpu_gpr[xSP];
    /* encoding here does not match that of c.lui */
    int imm =
        ONEBIT(insn, 2) << 5 |
        ONEBIT(insn, 3) << 7 |
        ONEBIT(insn, 5) << 6 |
        ONEBIT(insn, 6) << 4;

    if(ONEBIT(insn, 4))
        imm |= -1 << 8;

    tcg_gen_addi_tl(vd, vd, imm);
}

static void gen_clui(DC, uint16_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    int imm = rv_ci_imm(insn);

    switch(rd) {
        case 0: return;
        case 2: gen_caddi16sp(dc, insn); break;
        default: tcg_gen_movi_tl(cpu_gpr[rd], imm << 12);
    }
}

static void gen_cj(DC, uint16_t insn)
{
    int imm = rv_cj_imm(insn);
    gen_exit_tb(dc, EXIT_TB_DOWN, dc->pc + imm);
}

#ifdef TARGET_RISCV32
static void gen_cjal(DC, uint16_t insn)
{
    tcg_gen_movi_tl(cpu_gpr[xRA], dc->npc);
    gen_cj(dc, insn);
}
#endif

/* Opcode 100.10 (quadrant 2 func=100): add, rr move, j(al)r or ebreak */

static void gen_cmv(unsigned rd, unsigned rs)
{
    if(!rd) return;
    tcg_gen_mov_tl(cpu_gpr[rd], cpu_gpr[rs]);
}

static void gen_cjr(DC, unsigned rs)
{
    TCGv va = temp_new_andi(cpu_gpr[rs], -2);

    tcg_gen_mov_tl(cpu_pc, cpu_gpr[rs]);
    gen_exit_tb(dc, EXIT_TB_DOWN, ADDRESS_UNKNOWN);

    tcg_temp_free(va);
}

static void gen_cadd(unsigned rd, unsigned rs)
{
    if(!rd) return;
    tcg_gen_add_tl(cpu_gpr[rd], cpu_gpr[rd], cpu_gpr[rs]);
}

static void gen_cjalr(DC, unsigned rs)
{
    tcg_gen_movi_tl(cpu_gpr[xRA], dc->npc);
    gen_cjr(dc, rs);
}

static void gen_cebreak(DC)
{
    gen_illegal(dc); /* not implemented yet */
}

static void gen_cjmv(DC, uint16_t insn)
{
    unsigned rs1 = BITFIELD(insn, 11, 7);
    unsigned rs2 = BITFIELD(insn, 6, 2);

    if(ONEBIT(insn, 12)) {
        if(rs2)
            gen_cadd(rs1, rs2);
        else if(rs1)
            gen_cjalr(dc, rs1);
        else
            gen_cebreak(dc);
    } else {
        if(rs2)
            gen_cmv(rs1, rs2);
        else
            gen_cjr(dc, rs1);
    }
}

/* Compact branches. Predicate here is the opposite of what insn would
   imply, see comments for non-C gen_branch. */

static void gen_cbranch(DC, uint16_t insn, int predicate)
{
    unsigned rs = 8 + BITFIELD(insn, 9, 7);
    int imm = rv_cb_imm(insn);

    TCGv zero = tcg_const_tl(0);
    TCGv vs = cpu_gpr[rs];

    TCGLabel* skip = gen_new_label();
    tcg_gen_brcond_tl(predicate, vs, zero, skip);
    gen_exit_tb(dc, EXIT_TB_SIDE, dc->pc + imm);

    gen_set_label(skip);
    gen_exit_tb(dc, EXIT_TB_SIDE, dc->npc);
}

static void gen_cbeqz(DC, uint16_t insn)
{
    gen_cbranch(dc, insn, TCG_COND_NE);
}

static void gen_cbnez(DC, uint16_t insn)
{
    gen_cbranch(dc, insn, TCG_COND_EQ);
}

/* Register-based loads and stores.
   Beware thses functions look similar but differ slightly in imm encoding,
   registers (gpr, fpr; may be of different size) and load/store size. */

static unsigned rv_cl_regnum(uint16_t insn)
{
    return 8 + BITFIELD(insn, 4, 2);
}

static TCGv rv_cl_address(uint16_t insn, int imm)
{
    unsigned rs = 8 + BITFIELD(insn, 9, 7);
    TCGv va = tcg_temp_new();
    tcg_gen_addi_tl(va, cpu_gpr[rs], imm);
    return va;
}

static TCGv rv_cl_address_w(uint16_t insn)
{
    unsigned imm =
        (BITFIELD(insn, 12, 10) << 3) |
        (ONEBIT(insn, 6) << 2) |
        (ONEBIT(insn, 5) << 6);
    return rv_cl_address(insn, imm);
}

static TCGv rv_cl_address_d(uint16_t insn)
{
    unsigned imm =
        (BITFIELD(insn, 12, 10) << 3) |
        (ONEBIT(insn, 6) << 7) |
        (ONEBIT(insn, 5) << 6);
    return rv_cl_address(insn, imm);
}

static void gen_clw(DC, uint16_t insn)
{
    TCGv vr = cpu_gpr[rv_cl_regnum(insn)];
    TCGv va = rv_cl_address_w(insn);
    tcg_gen_qemu_ld32s(vr, va, dc->memidx);
    tcg_temp_free(va);
}

#ifndef TARGET_RISCV32
static void gen_cld(DC, uint16_t insn)
{
    TCGv vr = cpu_gpr[rv_cl_regnum(insn)];
    TCGv va = rv_cl_address_d(insn);
    tcg_gen_qemu_ld64(vr, va, dc->memidx);
    tcg_temp_free(va);
}
#endif

static void gen_cfld(DC, uint16_t insn)
{
    TCGf vd = cpu_fpr[rv_cl_regnum(insn)];
    TCGv va = rv_cl_address_d(insn);
    tcg_gen_qemu_ld64(vd, va, dc->memidx);
    tcg_temp_free(va);
}

static void gen_csw(DC, uint16_t insn)
{
    TCGv vr = cpu_gpr[rv_cl_regnum(insn)];
    TCGv va = rv_cl_address_w(insn);
    tcg_gen_qemu_st32(vr, va, dc->memidx);
    tcg_temp_free(va);
}

#ifndef TARGET_RISCV32
static void gen_csd(DC, uint16_t insn)
{
    TCGv vr = cpu_gpr[rv_cl_regnum(insn)];
    TCGv va = rv_cl_address_d(insn);
    tcg_gen_qemu_st64(vr, va, dc->memidx);
    tcg_temp_free(va);
}
#endif

static void gen_cfsd(DC, uint16_t insn)
{
    TCGf vr = cpu_fpr[rv_cl_regnum(insn)];
    TCGv va = rv_cl_address_d(insn);
    tcg_gen_qemu_st64(vr, va, dc->memidx);
    tcg_temp_free(va);
}

/* Stack-pointer-based loads and stores.

   Loads and stores here use different imm format.
   Loads to GPR (but not stores or loads to FPR) reject rd=0 */

static TCGv rv_ci_sp_address(unsigned off)
{
    TCGv va = tcg_temp_new();
    tcg_gen_addi_tl(va, cpu_gpr[xSP], off);
    return va;
}

static void gen_clwsp(DC, uint16_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned imm =
          (BITFIELD(insn, 3, 2) << 6)
        | (BITFIELD(insn, 6, 4) << 2)
        | (ONEBIT(insn, 12) << 5);

    if(!rd) return;

    TCGv vd = cpu_gpr[rd];
    TCGv va = rv_ci_sp_address(imm);
    tcg_gen_qemu_ld32s(vd, va, dc->memidx);
    tcg_temp_free(va);
}

#ifndef TARGET_RISCV32
static void gen_cldsp(DC, uint16_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned imm =
        (BITFIELD(insn, 4, 2) << 6) |
        (BITFIELD(insn, 6, 5) << 3) |
        (ONEBIT(insn, 12) << 5);

    if(!rd) return;

    TCGv vd = cpu_gpr[rd];
    TCGv va = rv_ci_sp_address(imm);
    tcg_gen_qemu_ld64(vd, va, dc->memidx);
    tcg_temp_free(va);
}
#endif

static void gen_cfldsp(DC, uint16_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned imm =
        (BITFIELD(insn, 4, 2) << 6) |
        (BITFIELD(insn, 6, 3) << 3) |
        (ONEBIT(insn, 12) << 5);

    TCGf vd = cpu_fpr[rd];
    TCGv va = rv_ci_sp_address(imm);
    tcg_gen_qemu_ld64(vd, va, dc->memidx);
    tcg_temp_free(va);
}

static unsigned rv_css_regnum(uint16_t insn)
{
    return BITFIELD(insn, 6, 2);
}

static TCGv rv_css_address(unsigned off)
{
    TCGv va = tcg_temp_new();
    tcg_gen_addi_tl(va, cpu_gpr[xSP], off);
    return va;
}

static TCGv rv_css_address_w(uint16_t insn)
{
    unsigned imm =
        (BITFIELD(insn, 12, 9) << 2) |
        (BITFIELD(insn,  8, 7) << 6);
    return rv_css_address(imm);
}

static TCGv rv_css_address_d(uint16_t insn)
{
    unsigned imm =
        (BITFIELD(insn, 12, 10) << 3) |
        (BITFIELD(insn,  9,  7) << 6);
    return rv_css_address(imm);
}

static void gen_cswsp(DC, uint16_t insn)
{
    TCGv vr = cpu_gpr[rv_css_regnum(insn)];
    TCGv va = rv_css_address_w(insn);
    tcg_gen_qemu_st32(vr, va, dc->memidx);
}

#ifndef TARGET_RISCV32
static void gen_csdsp(DC, uint16_t insn)
{
    TCGv vr = cpu_gpr[rv_css_regnum(insn)];
    TCGv va = rv_css_address_d(insn);
    tcg_gen_qemu_st64(vr, va, dc->memidx);
}
#endif

static void gen_cfsdsp(DC, uint16_t insn)
{
    TCGf vr = cpu_fpr[rv_css_regnum(insn)];
    TCGv va = rv_css_address_d(insn);
    tcg_gen_qemu_st64(vr, va, dc->memidx);
}

/* Immediate arithmetics */

static void gen_caddi(DC, uint16_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    int imm = BITFIELD(insn, 6, 2);

    if(ONEBIT(insn, 12))
        imm |= -1 << 5;

    if(!rd) return;

    TCGv vd = cpu_gpr[rd];
    tcg_gen_addi_tl(vd, vd, imm);
}

#ifndef TARGET_RISCV32
static void gen_caddiw(DC, uint16_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    int imm = BITFIELD(insn, 6, 2);

    if(ONEBIT(insn, 12))
        imm |= -1 << 5;

    if(!rd)
        return gen_illegal(dc);

    TCGv vd = cpu_gpr[rd];

    if(!imm)
        tcg_gen_ext32s_tl(vd, vd);
    else
        tcg_gen_addi_tl(vd, vd, imm);
}
#endif

static void gen_cslli(DC, uint16_t insn)
{
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned imm = BITFIELD(insn, 6, 2) | (ONEBIT(insn, 12) << 5);

    if(!rd) return;

    TCGv vd = cpu_gpr[rd];
    tcg_gen_shli_tl(vd, vd, imm);
}

/* Register-regiser arithmetics */

static void gen_cop_rr(DC, uint16_t insn, TCGv vd)
{
    unsigned op = ONEBIT(insn, 12) << 2 | BITFIELD(insn, 6, 5);
    unsigned rs = 8 + BITFIELD(insn, 4, 2);

    TCGv vs = cpu_gpr[rs];

    switch(op) {
        case /* 0.00 */ 0: tcg_gen_sub_tl(vd, vd, vs); break;
        case /* 0.01 */ 1: tcg_gen_xor_tl(vd, vd, vs); break;
        case /* 0.10 */ 2: tcg_gen_or_tl(vd, vd, vs); break;
        case /* 0.11 */ 3: tcg_gen_and_tl(vd, vd, vs); break;
        case /* 1.00 */ 4: tcg_gen_sub_tl(vd, vd, vs); break;
        case /* 1.01 */ 5: tcg_gen_add_tl(vd, vd, vs); break;
        default: gen_illegal(dc); return;
    }

    if(op & (1 << 2))
        tcg_gen_ext32s_tl(vd, vd);
}

static void gen_cop(DC, uint16_t insn)
{
    unsigned rd = 8 + BITFIELD(insn, 9, 7);
    unsigned imm = BITFIELD(insn, 6, 2) | (ONEBIT(insn, 12) << 5);
    int simm = BITFIELD(insn, 6, 2) | (ONEBIT(insn, 12) ? (-1 << 5) : 0);
    TCGv vd = cpu_gpr[rd];

    switch(BITFIELD(insn, 11, 10)) {
        case /* 00 */ 0: tcg_gen_shri_tl(vd, vd, imm); break;
        case /* 01 */ 1: tcg_gen_sari_tl(vd, vd, imm); break;
        case /* 10 */ 2: tcg_gen_andi_tl(vd, vd, simm); break;
        case /* 11 */ 3: gen_cop_rr(dc, insn, vd); break;
    }
}

static void gen_one_2byte_insn(DC, uint16_t insn)
{
    int funcop = (BITFIELD(insn, 15, 13) << 2) | BITFIELD(insn, 1, 0);

    switch(funcop) {
        case /* 000.00 */ 0x00: gen_addi4spn(dc, insn); break;
        case /* 010.00 */ 0x08: gen_clw(dc, insn); break;
        case /* 110.00 */ 0x18: gen_csw(dc, insn); break;
        case /* 001.00 */ 0x04: gen_cfld(dc, insn); break;
        case /* 101.00 */ 0x14: gen_cfsd(dc, insn); break;
        case /* 000.01 */ 0x01: gen_caddi(dc, insn); break;
        case /* 010.01 */ 0x09: gen_cli(dc, insn); break;
        case /* 011.01 */ 0x0D: gen_clui(dc, insn); break;
        case /* 100.01 */ 0x11: gen_cop(dc, insn); break;
        case /* 101.01 */ 0x15: gen_cj(dc, insn); break;
        case /* 110.01 */ 0x19: gen_cbeqz(dc, insn); break;
        case /* 111.01 */ 0x1D: gen_cbnez(dc, insn); break;
        case /* 000.10 */ 0x02: gen_cslli(dc, insn); break;
        case /* 001.10 */ 0x06: gen_cfldsp(dc, insn); break;
        case /* 010.10 */ 0x0A: gen_clwsp(dc, insn); break;
        case /* 100.10 */ 0x12: gen_cjmv(dc, insn); break;
        case /* 101.10 */ 0x16: gen_cfsdsp(dc, insn); break;
        case /* 110.10 */ 0x1A: gen_cswsp(dc, insn); break;
#ifdef TARGET_RISCV32
        case /* 001.01 */ 0x05: gen_cjal(dc, insn); break;
#else
        case /* 011.00 */ 0x0C: gen_cld(dc, insn); break;
        case /* 111.00 */ 0x1C: gen_csd(dc, insn); break;
        case /* 001.01 */ 0x05: gen_caddiw(dc, insn); break;
        case /* 011.10 */ 0x0E: gen_cldsp(dc, insn); break;
        case /* 111.10 */ 0x1E: gen_csdsp(dc, insn); break;
#endif
        default: gen_illegal(dc);
    }
}
