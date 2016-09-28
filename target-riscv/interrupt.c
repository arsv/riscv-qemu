/*
 *  RISC-V emulation helpers for qemu.
 *
 *  Author: Sagar Karandikar, sagark@eecs.berkeley.edu
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
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <signal.h>
#include "cpu.h"
#include "cpubits.h"

/*#define RISCV_DEBUG_INTERRUPT */

#ifdef CONFIG_USER_ONLY

bool riscv_cpu_has_work(CPUState *cs)
{
    return false;
}

bool riscv_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    return false;
}

void riscv_cpu_do_interrupt(CPUState *cs)
{
    cs->exception_index = EXCP_NONE;
}

#else

#define SSIP_IRQ (env->irq[0])
#define STIP_IRQ (env->irq[1])
#define MSIP_IRQ (env->irq[2])
#define TIMER_IRQ (env->irq[3])
#define HTIF_IRQ (env->irq[4])

/*
 * ctz in Spike returns 0 if val == 0, wrap helper
 */
static int ctz(target_ulong val)
{
    return val ? ctz64(val) : 0;
}

/*
 * Return RISC-V IRQ number if an interrupt should be taken, else -1.
 * Used in cpu-exec.c
 *
 * Adapted from Spike's processor_t::take_interrupt()
 */
static int cpu_riscv_hw_interrupts_pending(CPURISCVState *env)
{
    target_ulong pending_interrupts = env->mip & env->mie;

    target_ulong mie = get_field(env->mstatus, MSTATUS_MIE);
    target_ulong m_enabled = env->priv < PRV_M || (env->priv == PRV_M && mie);
    target_ulong enabled_interrupts = pending_interrupts &
                                      ~env->mideleg & -m_enabled;

    target_ulong sie = get_field(env->mstatus, MSTATUS_SIE);
    target_ulong s_enabled = env->priv < PRV_S || (env->priv == PRV_S && sie);
    enabled_interrupts |= pending_interrupts & env->mideleg & -s_enabled;

    if (enabled_interrupts) {
        target_ulong counted = ctz(enabled_interrupts);
        if (counted == IRQ_HOST) {
            /* we're handing it to the cpu now, so get rid of the qemu irq */
            qemu_irq_lower(HTIF_IRQ);
        } else if (counted == IRQ_M_TIMER) {
            /* we're handing it to the cpu now, so get rid of the qemu irq */
            qemu_irq_lower(TIMER_IRQ);
        } else if (counted == IRQ_S_TIMER || counted == IRQ_H_TIMER) {
            /* don't lower irq here */
        }
        return counted;
    } else {
        return 0; /* indicates no pending interrupt */
    }
}

bool riscv_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    if (interrupt_request & CPU_INTERRUPT_HARD) {
        RISCVCPU *cpu = RISCV_CPU(cs);
        CPURISCVState *env = &cpu->env;
        int interruptno = cpu_riscv_hw_interrupts_pending(env);
	int maxintno = EXCP_INTn - EXCP_INT0;
        if (interruptno >= 1 && interruptno <= maxintno) {
            cs->exception_index = EXCP_INT0 + interruptno;
            riscv_cpu_do_interrupt(cs);
            return true;
        }
    }
    return false;
}

#ifdef RISCV_DEBUG_INTERRUPT
static const char * const riscv_excp_names[12] = {
    "misaligned fetch",
    "fault fetch",
    "illegal instruction",
    "Breakpoint",
    "misaligned load",
    "fault load",
    "misaligned store",
    "fault store",
    "user_ecall",
    "supervisor_ecall",
    "hypervisor_ecall",
    "machine_ecall",
};

static const char * const riscv_interrupt_names[14] = {
    "",
    "S Soft interrupt",
    "H Soft interrupt",
    "M Soft interrupt",
    "",
    "S Timer interrupt",
    "H Timer interrupt",
    "M Timer interrupt",
    "",
    "S Ext interrupt",
    "H Ext interrupt",
    "M Ext interrupt",
    "COP interrupt",
    "Host interrupt"
};

static void riscv_log_interrupt(long cause)
{
    if (cause & 0x70000000) {
        int intno = cause & 0x0fffffff;
	if(intno < 0 || intno > sizeof(riscv_interrupt_names)/sizeof(char*))
		return;
	fprintf(stderr, "core   0: exception trap_%s, epc 0x" TARGET_FMT_lx "\n"
		, riscv_interrupt_names[intno], env->pc);
    } else if(cause >= 0 && cause < sizeof(riscv_excp_names)/sizeof(char*)) {
        fprintf(stderr, "core   0: exception trap_%s, epc 0x" TARGET_FMT_lx "\n"
                , riscv_excp_names[cause], env->pc);
    }
}
#endif     /* RISCV_DEBUG_INTERRUPT */

/* Translate qemu exception index into dcsr value */
static target_ulong excp_qemu_to_riscv(CPURISCVState* env, long excp)
{
    if(excp >= EXCP_INT0 && excp <= EXCP_INTn) {
        return (excp - EXCP_INT0) | (1L << 63);
    } else if(excp == EXCP_SYSCALL) {
        switch(env->priv) {
                case PRV_U: return RISCV_EXCP_U_ECALL;
                case PRV_S: return RISCV_EXCP_S_ECALL;
                case PRV_H: return RISCV_EXCP_H_ECALL;
                case PRV_M: return RISCV_EXCP_M_ECALL;
        }
    } else {
        switch(excp) {
            case EXCP_ILLEGAL: return RISCV_EXCP_ILLEGAL_INST;
            case EXCP_FAULT:   return RISCV_EXCP_INST_ACCESS_FAULT;
            /* TODO: load, misaligned AMO etc */
        }
    }

    return 0;
}

/*
 * Handle Traps
 *
 * Adapted from Spike's processor_t::take_trap.
 *
 */

void riscv_cpu_do_interrupt(CPUState *cs)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;

    long excp = cs->exception_index;
    target_ulong cause = excp_qemu_to_riscv(env, excp);

    if(!cause) {
        fprintf(stderr, "unexpected qemu exception %li\n", excp);
        return;
    }

    #ifdef RISCV_DEBUG_INTERRUPT
    riscv_log_interrupt(cause);
    #endif

    target_ulong backup_epc = env->pc;

    target_ulong bit = cause;
    target_ulong deleg = env->medeleg;

    int hasbadaddr =
        (cause == RISCV_EXCP_INST_ADDR_MIS) ||
        (cause == RISCV_EXCP_INST_ACCESS_FAULT) ||
        (cause == RISCV_EXCP_LOAD_ADDR_MIS) ||
        (cause == RISCV_EXCP_STORE_AMO_ADDR_MIS) ||
        (cause == RISCV_EXCP_LOAD_ACCESS_FAULT) ||
        (cause == RISCV_EXCP_STORE_AMO_ACCESS_FAULT);

    if (bit & ((target_ulong)1 << (TARGET_LONG_BITS - 1))) {
        deleg = env->mideleg, bit &= ~(1L << (TARGET_LONG_BITS - 1));
    }

    if (env->priv <= PRV_S && bit < 64 && ((deleg >> bit) & 1)) {
        /* handle the trap in S-mode */
        /* No need to check STVEC for misaligned - lower 2 bits cannot be set */
        env->pc = env->stvec;
        env->scause = cause;
        env->sepc = backup_epc;

        if (hasbadaddr) {
#ifdef RISCV_DEBUG_INTERRUPT
            fprintf(stderr, "core   0: badaddr 0x" TARGET_FMT_lx "\n", env->badaddr);
#endif
            env->sbadaddr = env->badaddr;
        }

        target_ulong s = env->mstatus;
        s = set_field(s, MSTATUS_SPIE, get_field(s, MSTATUS_UIE << env->priv));
        s = set_field(s, MSTATUS_SPP, env->priv);
        s = set_field(s, MSTATUS_SIE, 0);
        env->mstatus = s;
        riscv_set_privilege(env, PRV_S);
    } else {
        /* No need to check MTVEC for misaligned - lower 2 bits cannot be set */
        env->pc = env->mtvec;
        env->mepc = backup_epc;
        env->mcause = cause;

        if (hasbadaddr) {
#ifdef RISCV_DEBUG_INTERRUPT
            fprintf(stderr, "core   0: badaddr 0x" TARGET_FMT_lx "\n",
                    env->badaddr);
#endif
            env->mbadaddr = env->badaddr;
        }

        target_ulong s = env->mstatus;
        s = set_field(s, MSTATUS_MPIE, get_field(s, MSTATUS_UIE << env->priv));
        s = set_field(s, MSTATUS_MPP, env->priv);
        s = set_field(s, MSTATUS_MIE, 0);
        env->mstatus = s;
        riscv_set_privilege(env, PRV_M);
    }
    /* TODO yield load reservation  */

    cs->exception_index = EXCP_NONE; /* mark handled to qemu */
}

bool riscv_cpu_has_work(CPUState *cs)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;

    if (cs->interrupt_request & CPU_INTERRUPT_HARD) {
        int interruptno = cpu_riscv_hw_interrupts_pending(env);
	return !!(interruptno + 1);
    } else {
        return false;
    }
}

#endif
