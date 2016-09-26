/* Subsets F and D: single and double precision floating-point ops */

#ifndef IN_TRANSLATE_C
#error this file is a part of translate.c, do not compile separately
#endif

/* Like LOAD but writes to fpr instead of gpr.
   And f0 is a regular register, not a sink. */

static void gen_loadfp(DC, uint32_t insn)
{
    int imm = ((int32_t)insn >> 20);
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned rs = BITFIELD(insn, 19, 15);
    unsigned memidx = 0;   /* mmu, always 0 in linux-user mode */

    TCGv va = imm ? temp_new_rsum(cpu_gpr[rs], imm) : cpu_gpr[rs];
    TCGf vd = cpu_fpr[rd];

    switch(BITFIELD(insn, 14, 12)) {
        case /* 010 */ 2: tcg_gen_qemu_ld_i64(vd, va, memidx, MO_TEUL); break;
        case /* 011 */ 3: tcg_gen_qemu_ld_i64(vd, va, memidx, MO_TEQ); break;
        default: gen_illegal(dc);
    }

    if(imm) tcg_temp_free(va);
}

static void gen_storefp(DC, uint32_t insn)
{
    unsigned rs1 = BITFIELD(insn, 19, 15);  /* address */
    unsigned rs2 = BITFIELD(insn, 24, 20);  /* fpr to store */
    int32_t imm = BITFIELD(insn, 11, 7) |    /* 5 lower bits from insn */
           (((int32_t)insn >> 20) & ~0x1F);  /* 31:25 and 5 clear bits */
    unsigned memidx = 0;   /* mmu, always 0 in linux-user mode */

    TCGv va = imm ? temp_new_rsum(cpu_gpr[rs1], imm) : cpu_gpr[rs1];
    TCGf vs = cpu_fpr[rs2];

    switch(BITFIELD(insn, 14, 12)) {
        case /* 010 */ 2: tcg_gen_qemu_st_i64(vs, va, memidx, MO_TEUL); break;
        case /* 011 */ 3: tcg_gen_qemu_st_i64(vs, va, memidx, MO_TEQ); break;
        default: gen_illegal(dc);
    }

    if(imm) tcg_temp_free(va);
}

/* Fused multiply-add: rd = ±(rs1 x rs2 ± rs3)
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

    TCGf fd = cpu_fpr[rd];
    TCGf f1 = cpu_fpr[rs1];
    TCGf f2 = cpu_fpr[rs2];
    TCGf f3 = cpu_fpr[rs3];
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

static void gen_fsgnj(DC, TCGf fd, TCGf f1, TCGf f2, int rm, int floatwidth)
{
    TCGf sign = tcg_temp_new_i64();
    TCGf base = tcg_temp_new_i64();

    target_float signmask = ((target_float)1 << (floatwidth-1));
    target_float basemask = ((target_float)-1) & ~signmask;

    tcg_gen_andi_i64(sign, f2, signmask);
    tcg_gen_andi_i64(base, f1, basemask);

    switch(rm) {
        case /* 001 */ 1: tcg_gen_xori_i64(sign, sign, signmask);
        case /* 000 */ 0: tcg_gen_or_i64(fd, base, sign); break;
        case /* 010 */ 2: tcg_gen_xor_i64(fd, f1, sign); break;
        default: gen_illegal(dc);
    }

    tcg_temp_free_i64(base);
    tcg_temp_free_i64(sign);
}

static void gen_fminmax_s(DC, TCGf fd, TCGv_ptr ep, TCGf f1, TCGf f2, int rm)
{
    switch(rm) {
        case /* 000 */ 0: gen_helper_fmin_s(fd, ep, f1, f2); break;
        case /* 001 */ 1: gen_helper_fmax_s(fd, ep, f1, f2); break;
        default: gen_illegal(dc);
    }
}

static void gen_fminmax_d(DC, TCGf fd, TCGv_ptr ep, TCGf f1, TCGf f2, int rm)
{
    switch(rm) {
        case /* 000 */ 0: gen_helper_fmin_d(fd, ep, f1, f2); break;
        case /* 001 */ 1: gen_helper_fmax_d(fd, ep, f1, f2); break;
        default: gen_illegal(dc);
    }
}

static void gen_fcmp_s(DC, TCGv vd, TCGv_ptr ep, TCGf f1, TCGf f2, int rm)
{
    switch(rm) {
        case /* 000 */ 0: gen_helper_fle_s(vd, ep, f1, f2); break;
        case /* 001 */ 1: gen_helper_flt_s(vd, ep, f1, f2); break;
        case /* 010 */ 2: gen_helper_feq_s(vd, ep, f1, f2); break;
        default: gen_illegal(dc);
    }
}

static void gen_fcmp_d(DC, TCGv vd, TCGv_ptr ep, TCGf f1, TCGf f2, int rm)
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

static void gen_fcvt_xs(DC, TCGv vd, TCGv_ptr ep, TCGf f1, TCGv_i32 vm, int rs2)
{
    switch(rs2) {
        case /* 00000 */ 0: gen_helper_fcvt_w_s(vd, ep, f1, vm); break;
        case /* 00001 */ 1: gen_helper_fcvt_wu_s(vd, ep, f1, vm); break;
#ifndef TARGET_RISCV32
        case /* 00010 */ 2: gen_helper_fcvt_l_s(vd, ep, f1, vm); break;
        case /* 00011 */ 3: gen_helper_fcvt_lu_s(vd, ep, f1, vm); break;
#endif
        default: gen_illegal(dc);
    }
}

static void gen_fcvt_xd(DC, TCGv vd, TCGv_ptr ep, TCGf f1, TCGv_i32 vm, int rs2)
{
    switch(rs2) {
        case /* 00000 */ 0: gen_helper_fcvt_w_d(vd, ep, f1, vm); break;
        case /* 00001 */ 1: gen_helper_fcvt_wu_d(vd, ep, f1, vm); break;
#ifndef TARGET_RISCV32
        case /* 00010 */ 2: gen_helper_fcvt_l_d(vd, ep, f1, vm); break;
        case /* 00011 */ 3: gen_helper_fcvt_lu_d(vd, ep, f1, vm); break;
#endif
        default: gen_illegal(dc);
    }
}

static void gen_fcvt_sx(DC, TCGf fd, TCGv_ptr ep, TCGv v1, TCGv_i32 vm, int rs2)
{
    switch(rs2) {
        case /* 00000 */ 0: gen_helper_fcvt_s_w(fd, ep, v1, vm); break;
        case /* 00001 */ 1: gen_helper_fcvt_s_wu(fd, ep, v1, vm); break;
#ifndef TARGET_RISCV32
        case /* 00010 */ 2: gen_helper_fcvt_s_l(fd, ep, v1, vm); break;
        case /* 00011 */ 3: gen_helper_fcvt_s_lu(fd, ep, v1, vm); break;
#endif
        default: gen_illegal(dc);
    }
}

static void gen_fcvt_dx(DC, TCGf fd, TCGv_ptr ep, TCGv v1, TCGv_i32 vm, int rs2)
{
    switch(rs2) {
        case /* 00000 */ 0: gen_helper_fcvt_d_w(fd, ep, v1, vm); break;
        case /* 00001 */ 1: gen_helper_fcvt_d_wu(fd, ep, v1, vm); break;
#ifndef TARGET_RISCV32
        case /* 00010 */ 2: gen_helper_fcvt_d_l(fd, ep, v1, vm); break;
        case /* 00011 */ 3: gen_helper_fcvt_d_lu(fd, ep, v1, vm); break;
#endif
        default: gen_illegal(dc);
    }
}

/* FMV are just moves, but they share major opcodes with FCLASS
   and those do need helpers. */

static void gen_fmv_xs(DC, TCGv vd, TCGf f1, unsigned rm)
{
    switch(rm) {
#ifdef TARGET_RISCV32
        case /* 000 */ 0: tcg_gen_trunc_i64_tl(vd, f1); break;
#else
        case /* 000 */ 0: tcg_gen_ext32s_tl(vd, f1); break;
#endif
        case /* 001 */ 1: gen_helper_fclass_s(vd, f1); break;
        default: gen_illegal(dc);
    }
}

static void gen_fmv_xd(DC, TCGv vd, TCGf f1, unsigned rm)
{
    switch(rm) {
#ifndef TARGET_RISCV32
        case /* 000 */ 0: tcg_gen_mov_tl(vd, f1); break;
#endif
        case /* 001 */ 1: gen_helper_fclass_d(vd, f1); break;
        default: gen_illegal(dc);
    }
}

static void gen_fmv_sx(DC, TCGf fd, TCGv v1, unsigned rm)
{
    switch(rm) {
#ifdef TARGET_RISCV32
        case /* 000 */ 0: tcg_gen_ext_i32_i64(fd, v1); break;
#else
        case /* 000 */ 0: tcg_gen_ext32s_tl(fd, v1); break;
#endif
        default: gen_illegal(dc);
    }
}

static void gen_fmv_dx(DC, TCGf fd, TCGv v1, unsigned rm)
{
    switch(rm) {
#ifndef TARGET_RISCV32
        case /* 000 */ 0: tcg_gen_mov_tl(fd, v1); break;
#endif
        default: gen_illegal(dc);
    }
}

static void gen_fcvt_sd(DC, TCGf fd, TCGv_ptr ep, TCGf f1, TCGv_i32 vm, int rs2)
{
    switch(rs2) {
        case /* 00001 */ 1: gen_helper_fcvt_s_d(fd, ep, f1, vm); break;
        default: gen_illegal(dc);
    }
}

static void gen_fcvt_ds(DC, TCGf fd, TCGv_ptr ep, TCGf f1, TCGv_i32 vm, int rs2)
{
    switch(rs2) {
        case /* 00000 */ 0: gen_helper_fcvt_d_s(fd, ep, f1, vm); break;
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

    TCGf fd = cpu_fpr[rd];
    TCGf f1 = cpu_fpr[rs1];
    TCGf f2 = cpu_fpr[rs2];

    TCGv vd = cpu_gpr[rd];
    TCGv v1 = cpu_gpr[rs1];
    TCGv_i32 vm = tcg_const_i32(rm);

    switch(func) {
        /* Single-precision */
        case /* 0000000 */ 0x00: gen_helper_fadd_s(fd, ep, f1, f2, vm); break;
        case /* 0000100 */ 0x04: gen_helper_fsub_s(fd, ep, f1, f2, vm); break;
        case /* 0001000 */ 0x08: gen_helper_fmul_s(fd, ep, f1, f2, vm); break;
        case /* 0001100 */ 0x0C: gen_helper_fdiv_s(fd, ep, f1, f2, vm); break;
        case /* 0101100 */ 0x2C: gen_helper_fsqrt_s(fd, ep, f1, vm); break;
        case /* 0010000 */ 0x10: gen_fsgnj(dc, fd, f1, f2, rm, 32); break;
        case /* 0010100 */ 0x14: gen_fminmax_s(dc, fd, ep, f1, f2, rm); break;
        case /* 1010000 */ 0x50: gen_fcmp_s(dc, vd, ep, f1, f2, rm); break;
        /* Double-precision */
        case /* 0000001 */ 0x01: gen_helper_fadd_d(fd, ep, f1, f2, vm); break;
        case /* 0000101 */ 0x05: gen_helper_fsub_d(fd, ep, f1, f2, vm); break;
        case /* 0001001 */ 0x09: gen_helper_fmul_d(fd, ep, f1, f2, vm); break;
        case /* 0001101 */ 0x0D: gen_helper_fdiv_d(fd, ep, f1, f2, vm); break;
        case /* 0101101 */ 0x2D: gen_helper_fsqrt_d(fd, ep, f1, vm); break;
        case /* 0010001 */ 0x11: gen_fsgnj(dc, fd, f1, f2, rm, 64); break;
        case /* 0010101 */ 0x15: gen_fminmax_d(dc, fd, ep, f1, f2, rm); break;
        case /* 1010001 */ 0x51: gen_fcmp_d(dc, vd, ep, f1, f2, rm); break;
        /* Single-Double conversion */
        case /* 0100000 */ 0x20: gen_fcvt_sd(dc, fd, ep, f1, vm, rs2); break;
        case /* 0100001 */ 0x21: gen_fcvt_ds(dc, fd, ep, f1, vm, rs2); break;
        /* Float-integer conversion */
        case /* 1100000 */ 0x60: gen_fcvt_xs(dc, vd, ep, f1, vm, rs2); break;
        case /* 1101000 */ 0x68: gen_fcvt_sx(dc, fd, ep, v1, vm, rs2); break;
        case /* 1100001 */ 0x61: gen_fcvt_xd(dc, vd, ep, f1, vm, rs2); break;
        case /* 1101001 */ 0x69: gen_fcvt_dx(dc, fd, ep, v1, vm, rs2); break;
        /* Float-Integer moves (and fclassify) */
        case /* 1110000 */ 0x70: gen_fmv_xs(dc, vd, f1, rm); break;
        case /* 1111000 */ 0x78: gen_fmv_sx(dc, fd, v1, rm); break;
        case /* 1110001 */ 0x71: gen_fmv_xd(dc, vd, f1, rm); break;
        case /* 1111001 */ 0x79: gen_fmv_dx(dc, fd, v1, rm); break;
        default: gen_illegal(dc);
    }

    tcg_temp_free_i32(vm);
}
