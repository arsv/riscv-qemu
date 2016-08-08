#include "qemu/osdep.h"
#include "exception.h"
#include "exec/helper-proto.h"
#include "exec/exec-all.h"

#define BITFIELD(src, end, start) \
            (((src) >> start) & ((1 << (end - start + 1)) - 1))

#define ENV CPURISCVState* env

/* Since FP is done via softfloat, FP status is actually softfloat status.
   However, softfloat bit values do not match those of RISC-V fflags,
   so some translation is necessary.

   Keeping a shadow fflags variable in ENV makes no sense and would not
   simplify any of the following functions. */

static unsigned rv_rcsr_fflags(ENV)
{
    signed char flags = env->fpstatus.float_exception_flags;
    unsigned ret = 0;

    if(flags & float_flag_invalid)
        ret |= (1<<4); /* NV */
    if(flags & float_flag_divbyzero)
        ret |= (1<<3); /* DZ */
    if(flags & float_flag_overflow)
        ret |= (1<<2); /* OF */
    if(flags & float_flag_underflow)
        ret |= (1<<1); /* UF */
    if(flags & float_flag_inexact)
        ret |= (1<<0); /* NX */

    return ret;
}

static void rv_wcsr_fflags(ENV, unsigned val)
{
    signed char flags = 0;

    if(val & (1<<4)) /* NV */
        flags |= float_flag_invalid;
    if(val & (1<<3)) /* DZ */
        flags |= float_flag_divbyzero;
    if(val & (1<<2)) /* OF */
        flags |= float_flag_overflow;
    if(val & (1<<1)) /* UF */
        flags |= float_flag_underflow;
    if(val & (1<<0)) /* NX */
        flags |= float_flag_inexact;

    env->fpstatus.float_exception_flags = flags;
}

/* Rounding mode register merely stores the value, there are no side-effects
   on write. Translation into fpstatus happens in FP op helpers.

   Note env->fpstatus.float_rounding_mode holds current effective value
   for each given instruction, which may be static (per-insn) or dynamic (frm),
   so frm itself must be stored somewhere else. */

static void rv_wcsr_frm(ENV, unsigned val)
{
    env->frm = val & 7;
}

static unsigned rv_rcsr_fcsr(ENV)
{
    return rv_rcsr_fflags(env) | (env->frm << 5);
}

static void rv_wcsr_fcsr(ENV, unsigned val)
{
    rv_wcsr_fflags(env, BITFIELD(val, 4, 0));
    rv_wcsr_frm(env, BITFIELD(val, 7, 5));
}

/* Primary CSR tables.

   Omitting read-only registers from the write table ensures EXCP_ILLEGAL
   on any write attempt. There are not other checks for ro/rw status. */

static target_ulong rv_csr_read(ENV, unsigned csr)
{
    switch(csr) {
        case 0x001: return rv_rcsr_fflags(env);
        case 0x002: return env->frm;
        case 0x003: return rv_rcsr_fcsr(env);
        default: raise_env_exception(env, EXCP_ILLEGAL);
    }
}

static void rv_csr_write(ENV, unsigned csr, target_ulong val)
{
    switch(csr) {
        case 0x001: rv_wcsr_fflags(env, val); break;
        case 0x002: rv_wcsr_frm(env, val); break;
        case 0x003: rv_wcsr_fcsr(env, val); break;
        default: raise_env_exception(env, EXCP_ILLEGAL);
    }
}

#define RW 1
#define RS 2
#define RC 3

/* insn is the instruction being handled here, major opcode SYSTEM. */

void HELPER(csr)(ENV, uint32_t insn)
{
    unsigned rs = BITFIELD(insn, 19, 15);
    unsigned func = BITFIELD(insn, 14, 12);
    unsigned rd = BITFIELD(insn, 11, 7);
    unsigned csr = BITFIELD(insn, 31, 20);

    unsigned regmode = BITFIELD(csr, 11, 10);
    unsigned cpumode = 0; /* linux-user always runs in user mode */

    unsigned regop = BITFIELD(func, 1, 0); /* RW, RS or RC */
    unsigned immrs = BITFIELD(func, 2, 2);

    if(cpumode < regmode)
        goto illegal;

    if(regop == RW && !rd) /* CSRRW(I) x0 are silent nops */
        return;

    target_ulong val = rv_csr_read(env, csr);

    if(regop != RW && !rs) /* CSRRS(I)/CSRRC(i) with rs=x0 skip writes */
        return;

    target_ulong mask = immrs ? rs : env->gpr[rs];

    switch(regop) {
        case RW: val = mask; break;         /* CSRRW */
        case RS: val = val |  mask; break;  /* CSRRS */
        case RC: val = val & ~mask; break;  /* CSRRC */
        default: goto illegal;
    }

    rv_csr_write(env, csr, val);

    return;

illegal:
    raise_env_exception(env, EXCP_ILLEGAL);
}
