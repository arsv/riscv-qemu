/* Common and/or nonspecific utility routines for op handlers */

#ifndef IN_TRANSLATE_C
#error part of translate.c, do not compile separately
#endif

static void gen_exception(DC, unsigned int excp)
{
    TCGv_i32 tmp = tcg_const_i32(excp);
    gen_helper_exception(cpu_env, tmp);
    tcg_temp_free_i32(tmp);
}

static void gen_illegal(DC)
{
    tcg_gen_movi_tl(cpu_pc, dc->pc);
    gen_exception(dc, EXCP_ILLEGAL);

    /* Stop translating at the first illegal instruction encountered.
       It will cause exception and block exit anyway, so there is no
       point in proceeding.

       This is wrong in case exception is raised conditionally at runtime
       *and* the other branch is not a jump, like AMO for instance.
       AMOs should raise something other than EXCP_ILLEGAL anyway. */

    dc->jump = true;
}

/* TB terminates on the first branch or jump encountered, so there's
   always at most two exits from the TB: sideways (taken branch) and
   down (anything else). Exceptions terminate TBs as well but need no
   gen_exit_tb because raise_exception does that itself.

   A normal exit in gen_intermediate_code is handled like a jump
   to the next pc.

   Before exiting, each TB must write the address of the next insn
   to env->pc to allow cpu_loop to continue. In most cases the address
   is known statically. However, in jump-to-register insns (jr, jalr,
   cjr, cjalr) it is not, so those instructions handle pc writes
   on their own and pass ADDRESS_UNKNOWN here. */

static bool rv_can_jump_to(DC, target_ulong dest)
{
    return (dc->pc & TARGET_PAGE_MASK) == (dest & TARGET_PAGE_MASK);
}

static void gen_exit_tb(DC, int n, target_ulong dest)
{
    if(dest != ADDRESS_UNKNOWN)
        tcg_gen_movi_tl(cpu_pc, dest);
    /* else jump target is determined at runtime and the proper
       tcg_gen_mov*(cpu_pc, ...) has already been called */

    if(dc->singlestep) {
        gen_exception(dc, EXCP_DEBUG);
    } else if(dest != ADDRESS_UNKNOWN && rv_can_jump_to(dc, dest)) {
        tcg_gen_goto_tb(n);
        tcg_gen_exit_tb((uintptr_t)dc->tb + n);
    } else {
        tcg_gen_exit_tb(0);
    }

    if(n == EXIT_TB_DOWN)
        dc->jump = true;
}

static void gen_breakpoint(DC)
{
    tcg_gen_movi_tl(cpu_pc, dc->pc);
    gen_exception(dc, EXCP_DEBUG);
    dc->jump = true;
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

static TCGv temp_local_new_ext32s(TCGv vs)
{
    TCGv vx = tcg_temp_local_new();
    tcg_gen_ext32s_tl(vx, vs);
    return vx;
}

static TCGv temp_new_ext32u(TCGv vs)
{
    TCGv vx = tcg_temp_new();
    tcg_gen_ext32u_tl(vx, vs);
    return vx;
}

static TCGv temp_local_new_ext32u(TCGv vs)
{
    TCGv vx = tcg_temp_local_new();
    tcg_gen_ext32u_tl(vx, vs);
    return vx;
}

static TCGv temp_new_andi(TCGv vs, target_long mask)
{
    TCGv vr = tcg_temp_new();
    tcg_gen_andi_tl(vr, vs, mask);
    return vr;
}
