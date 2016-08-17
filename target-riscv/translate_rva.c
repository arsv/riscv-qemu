/* Subset A: atomic memory ops */

#ifndef IN_TRANSLATE_C
#error this file is a part of translate.c, do not compile separately
#endif

/* In linux-user mode, AMOs stop all virtual cores to ensure exclusivity.
   See atomic.c, and linux-user/main.c for EXCP_ATOMIC. All decoding gets
   relegated there as well.

   In system mode, only one core is running at any given time, so AMOs are
   just regular TCG ops. */

static void gen_amo(DC, uint32_t insn)
{
    tcg_gen_movi_i32(cpu_amoinsn, insn);
    tcg_gen_movi_tl(cpu_pc, dc->pc);
    gen_exception(dc, EXCP_ATOMIC);
    dc->jump = true;
}
