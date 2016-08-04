#include "qemu/osdep.h"
#include "exception.h"
#include "exec/helper-proto.h"
#include "exec/exec-all.h"

#define BITFIELD(src, end, start) \
            (((src) >> start) & ((1 << (end - start + 1)) - 1))

static target_ulong rv_csr_read(CPURISCVState *env, unsigned csr)
{
    switch(csr) {
        default: raise_env_exception(env, EXCP_ILLEGAL);
    }
}

static void rv_csr_write(CPURISCVState *env, unsigned csr, target_ulong val)
{
    switch(csr) {
        default: raise_env_exception(env, EXCP_ILLEGAL);
    }
}

#define RW 1
#define RS 2
#define RC 3

/* insn is the instruction being handled here, major opcode SYSTEM. */

void HELPER(csr)(CPURISCVState *env, uint32_t insn)
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
