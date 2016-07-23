#ifndef TARGET_CPU_H
#define TARGET_CPU_H

static inline void cpu_clone_regs(CPURISCVState *env, target_ulong newsp)
{
    if (newsp)
        env->gpr[xSP] = newsp;

    /* stub */
}

static inline void cpu_set_tls(CPURISCVState *env, target_ulong newtls)
{
    /* stub */
}

#endif
