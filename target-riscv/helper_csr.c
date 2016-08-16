/*
 * RISC-V control and status register helpers
 *
 * Copyright (c) 2016 Alex Suykov <alex.suykov@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

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

static unsigned rv_get_fflags(ENV)
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

static void rv_set_fflags(ENV, unsigned val)
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

static void rv_set_frm(ENV, unsigned val)
{
    env->frm = val & 7;
}

static unsigned rv_get_fcsr(ENV)
{
    return rv_get_fflags(env) | (env->frm << 5);
}

static void rv_set_fcsr(ENV, unsigned val)
{
    rv_set_fflags(env, BITFIELD(val, 4, 0));
    rv_set_frm(env, BITFIELD(val, 7, 5));
}

/* Primary CSR tables.
   RW registers should appear in both tables, RO only in rv_get_csr().
   There are not other checks for RO/RW status. */

static target_ulong rv_get_csr(ENV, unsigned csr)
{
    switch(csr) {
        case 0x001: return rv_get_fflags(env);
        case 0x002: return env->frm;
        case 0x003: return rv_get_fcsr(env);
        default: raise_exception(env, EXCP_ILLEGAL);
    }
}

static void rv_set_csr(ENV, unsigned csr, target_ulong val)
{
    switch(csr) {
        case 0x001: rv_set_fflags(env, val); break;
        case 0x002: rv_set_frm(env, val); break;
        case 0x003: rv_set_fcsr(env, val); break;
        default: raise_exception(env, EXCP_ILLEGAL);
    }
}

#define RW 1
#define RS 2
#define RC 3

/* Unlike most other helpers, this one gets non-decoded isns.
   Some decoding would happen here anyway, and there's a lot
   of common code for the six CSR ops, so no point in doing
   much in translate.c */

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
    target_ulong mask = immrs ? rs : env->gpr[rs];
    target_ulong val;

    if(cpumode < regmode)
        goto illegal;

    if(regop == RW && !rd)
        val = 0; /* CSRRW(I) x0 do not read the reg */
    else
        val = rv_get_csr(env, csr);

    if(rd) env->gpr[rd] = val;

    if(regop != RW && !rs)
        return; /* CSRRS(I)/CSRRC(i) with rs=x0 skip writes */

    switch(regop) {
        case RW: val = mask; break;         /* CSRRW */
        case RS: val = val |  mask; break;  /* CSRRS */
        case RC: val = val & ~mask; break;  /* CSRRC */
        default: goto illegal;
    }

    rv_set_csr(env, csr, val);

    return;

illegal:
    raise_exception(env, EXCP_ILLEGAL);
}
