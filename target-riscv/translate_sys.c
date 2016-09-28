/* Syscalls and priveleged ops */

#ifndef IN_TRANSLATE_C
#error this file is a part of translate.c, do not compile separately
#endif

/* Memory FENCE and FENCE.I with i-cache flush.
   rd, rs and imm ignored as per spec. */

static void gen_fence_i(DC)
{
#ifndef CONFIG_USER_ONLY
    gen_helper_flush(cpu_env);
    gen_exit_tb(dc, EXIT_TB_DOWN, ADDRESS_UNKNOWN);
#endif
}

static void gen_miscmem(DC, uint32_t insn)
{
    switch(BITFIELD(insn, 14, 12))
    {
        case 0: break; /* FENCE, nop */
        case 1: gen_fence_i(dc); break;
        default: gen_illegal(dc);
    }
}

#ifndef CONFIG_USER_ONLY

static void gen_sret(DC)
{
    gen_helper_sret(cpu_env);
    gen_exit_tb(dc, EXIT_TB_DOWN, ADDRESS_UNKNOWN);
}

static void gen_mret(DC)
{
    gen_helper_mret(cpu_env);
    gen_exit_tb(dc, EXIT_TB_DOWN, ADDRESS_UNKNOWN);
}

#endif

static void gen_ecall(DC)
{
    gen_excp_exit(dc, EXCP_SYSCALL);
}

static void gen_ebreak(DC)
{
    gen_excp_exit(dc, EXCP_DEBUG);
}

static void gen_priv(DC, uint32_t insn)
{
    switch(BITFIELD(insn, 31, 20)) {
        case 0x000: gen_ecall(dc); break;
        case 0x001: gen_ebreak(dc); break;
#ifndef CONFIG_USER_ONLY
        case 0x102: gen_sret(dc); break;
        case 0x302: gen_mret(dc); break;
        case 0x105: /* WFI, nop for now */ break;
        case 0x104: gen_helper_flush(cpu_env); break;
#endif
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
