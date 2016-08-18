/* Subset M: division and multiplication. */

#ifndef IN_TRANSLATE_C
#error this file is a part of translate.c, do not compile separately
#endif

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
   The final addition is two words long. */

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
    tcg_gen_xori_tl(notresl, resl, -1);   /* notresl = ~resl */
    tcg_gen_addi_tl(resl, notresl, 1);    /* resl = ~resl + 1 */

    /* set vd = ~resh + carry */
    tcg_gen_xori_tl(vd, resh, -1);            /* vd = ~resh */
    tcg_gen_brcond_tl(TCG_COND_LTU, notresl, resl, done);
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
    gen_div_overcheck(done, vd, vs1, vs2, 0);
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
    gen_div_overcheck(done, vd, vs1, vs2, 1);
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

#ifndef TARGET_RISCV32

static void gen_mulw(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv sink = tcg_temp_new();

    /* no need to sign-extend anything there */
    tcg_gen_muls2_tl(vd, sink, vs1, vs2);
    tcg_gen_ext32s_tl(vd, vd);

    tcg_temp_free(sink);
}

/* OP32 division routines carry temps across branches!
   All temps must be local, TCG does not like it otherwise. */

static void gen_divw(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv vx1 = temp_local_new_ext32s(vs1);
    TCGv vx2 = temp_local_new_ext32s(vs2);
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
    TCGv vx1 = temp_local_new_ext32u(vs1);
    TCGv vx2 = temp_local_new_ext32u(vs2);
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vx1, vx2, 0);
    tcg_gen_divu_tl(vd, vx1, vx2);

    tcg_temp_free(vx2);
    tcg_temp_free(vx1);

    gen_set_label(done);
    tcg_gen_ext32s_tl(vd, vd);
}

static void gen_remw(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv vx1 = temp_local_new_ext32s(vs1);
    TCGv vx2 = temp_local_new_ext32s(vs2);
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vx1, vx2, 1);
    /* gen_div_overcheck not needed, DIVW cannot overflow? */
    tcg_gen_rem_tl(vd, vx1, vx2);

    tcg_temp_free(vx2);
    tcg_temp_free(vx1);

    gen_set_label(done);
    tcg_gen_ext32s_tl(vd, vd);
}

static void gen_remuw(TCGv vd, TCGv vs1, TCGv vs2)
{
    TCGv vx1 = temp_local_new_ext32u(vs1);
    TCGv vx2 = temp_local_new_ext32u(vs2);
    TCGLabel* done = gen_new_label();

    gen_div_zerocheck(done, vd, vx1, vx2, 1);
    tcg_gen_rem_tl(vd, vx1, vx2);

    tcg_temp_free(vx2);
    tcg_temp_free(vx1);

    gen_set_label(done);
    tcg_gen_ext32s_tl(vd, vd); /* sign-extend even in U case! */
}

#endif
