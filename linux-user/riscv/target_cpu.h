#ifndef TARGET_CPU_H
#define TARGET_CPU_H

static inline void cpu_clone_regs(CPURISCVState *env, target_ulong newsp)
{
    if (newsp)
        env->sp = newsp; /* x30 (sp) stack pointer */

    /* stub */
}

static inline void cpu_set_tls(CPURISCVState *env, target_ulong newtls)
{
    /* stub */
}

#endif
