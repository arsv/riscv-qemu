/*
 *  RISC-V emulation helpers for qemu.
 *
 *  Author: Sagar Karandikar <sagark@eecs.berkeley.edu>
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

#ifndef CONFIG_USER_ONLY

/* get_physical_address - get the physical address for this virtual address
 *
 * Do a page table walk to obtain the physical address corresponding to a
 * virtual address. Returns 0 if the translation was successful
 *
 * Adapted from Spike's mmu_t::translate and mmu_t::walk
 *
 */
static int get_physical_address(CPURISCVState *env, hwaddr *physical,
                                int *prot, target_ulong address,
                                int access_type, int mmu_idx)
{
    /* NOTE: the env->pc value visible here will not be
     * correct, but the value visible to the exception handler
     * (riscv_cpu_do_interrupt) is correct */

    *prot = 0;
    CPUState *cs = CPU(riscv_env_get_cpu(env));

    target_ulong mode = env->priv;
    if (access_type != MMU_INST_FETCH) {
        if (get_field(env->mstatus, MSTATUS_MPRV)) {
            mode = get_field(env->mstatus, MSTATUS_MPP);
        }
    }
    if (get_field(env->mstatus, MSTATUS_VM) == VM_MBARE) {
        mode = PRV_M;
    }

    /* check to make sure that mmu_idx and mode that we get matches */
    if (unlikely(mode != mmu_idx)) {
        fprintf(stderr, "MODE, mmu_idx mismatch\n");
        exit(1);
    }

    if (mode == PRV_M) {
        target_ulong msb_mask = (2UL << (TARGET_LONG_BITS - 1)) - 1;
                                        /*0x7FFFFFFFFFFFFFFF; */
        *physical = address & msb_mask;
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
        return TRANSLATE_SUCCESS;
    }

    target_ulong addr = address;
    int supervisor = mode == PRV_S;
    int pum = get_field(env->mstatus, MSTATUS_PUM);
    int mxr = get_field(env->mstatus, MSTATUS_MXR);

    int levels, ptidxbits, ptesize;
    switch (get_field(env->mstatus, MSTATUS_VM)) {
    case VM_SV32:
      levels = 2;
      ptidxbits = 10;
      ptesize = 4;
      break;
    case VM_SV39:
      levels = 3;
      ptidxbits = 9;
      ptesize = 8;
      break;
    case VM_SV48:
      levels = 4;
      ptidxbits = 9;
      ptesize = 8;
      break;
    default:
      printf("unsupported MSTATUS_VM value\n");
      exit(1);
    }

    int va_bits = PGSHIFT + levels * ptidxbits;
    target_ulong mask = (1L << (TARGET_LONG_BITS - (va_bits - 1))) - 1;
    target_ulong masked_msbs = (addr >> (va_bits - 1)) & mask;
    if (masked_msbs != 0 && masked_msbs != mask) {
        return TRANSLATE_FAIL;
    }

    target_ulong base = env->sptbr << PGSHIFT;
    int ptshift = (levels - 1) * ptidxbits;
    int i;
    for (i = 0; i < levels; i++, ptshift -= ptidxbits) {
        target_ulong idx = (addr >> (PGSHIFT + ptshift)) &
                           ((1 << ptidxbits) - 1);

        /* check that physical address of PTE is legal */
        target_ulong pte_addr = base + idx * ptesize;

        /* PTE must reside in memory */
        if (pte_addr < RISCV_DRAM_BASE) {
            printf("PTE was not in DRAM region\n");
            exit(1);
            break;
        }

        target_ulong pte = ldq_phys(cs->as, pte_addr);
        target_ulong ppn = pte >> PTE_PPN_SHIFT;

        if (PTE_TABLE(pte)) { /* next level of page table */
            base = ppn << PGSHIFT;
        } else if ((pte & PTE_U) ? supervisor && pum : !supervisor) {
            break;
        } else if (!(pte & PTE_V) || (!(pte & PTE_R) && (pte & PTE_W))) {
            break;
        } else if (access_type == MMU_INST_FETCH ? !(pte & PTE_X) :
                  access_type == MMU_DATA_LOAD ?  !(pte & PTE_R) &&
                  !(mxr && (pte & PTE_X)) : !((pte & PTE_R) && (pte & PTE_W))) {
            break;
        } else {
            /* set accessed and possibly dirty bits.
               we only put it in the TLB if it has the right stuff */
            stq_phys(cs->as, pte_addr, ldq_phys(cs->as, pte_addr) | PTE_A |
                    ((access_type == MMU_DATA_STORE) * PTE_D));

            /* for superpage mappings, make a fake leaf PTE for the TLB's
               benefit. */
            target_ulong vpn = addr >> PGSHIFT;
            *physical = (ppn | (vpn & ((1L << ptshift) - 1))) << PGSHIFT;

            /* we do not give all prots indicated by the PTE
             * this is because future accesses need to do things like set the
             * dirty bit on the PTE
             *
             * at this point, we assume that protection checks have occurred */
            if (supervisor) {
                if ((pte & PTE_X) && access_type == MMU_INST_FETCH) {
                    *prot |= PAGE_EXEC;
                } else if ((pte & PTE_W) && access_type == MMU_DATA_STORE) {
                    *prot |= PAGE_WRITE;
                } else if ((pte & PTE_R) && access_type == MMU_DATA_LOAD) {
                    *prot |= PAGE_READ;
                } else {
                    printf("err in translation prots");
                    exit(1);
                }
            } else {
                if ((pte & PTE_X) && access_type == MMU_INST_FETCH) {
                    *prot |= PAGE_EXEC;
                } else if ((pte & PTE_W) && access_type == MMU_DATA_STORE) {
                    *prot |= PAGE_WRITE;
                } else if ((pte & PTE_R) && access_type == MMU_DATA_LOAD) {
                    *prot |= PAGE_READ;
                } else {
                    printf("err in translation prots");
                    exit(1);
                }
            }
            return TRANSLATE_SUCCESS;
        }
    }
    return TRANSLATE_FAIL;
}

static void raise_mmu_exception(CPURISCVState *env, target_ulong address,
                                MMUAccessType access_type)
{
    CPUState *cs = CPU(riscv_env_get_cpu(env));

    env->sbadaddr = address;
    cs->exception_index = EXCP_FAULT;
}

#endif

int riscv_cpu_handle_mmu_fault(CPUState *cs, vaddr address,
        int access_type, int mmu_idx)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;
#ifndef CONFIG_USER_ONLY
    hwaddr physical;
    physical = 0; /* stop gcc complaining */
    int prot;
#endif
    int ret = -1;

    qemu_log_mask(CPU_LOG_MMU,
            "%s pc " TARGET_FMT_lx " ad %" VADDR_PRIx " access_type %d mmu_idx \
             %d\n", __func__, env->pc, address, access_type, mmu_idx);

#ifndef CONFIG_USER_ONLY
    ret = get_physical_address(env, &physical, &prot, address, access_type,
                               mmu_idx);
    qemu_log_mask(CPU_LOG_MMU,
            "%s address=%" VADDR_PRIx " ret %d physical " TARGET_FMT_plx
             " prot %d\n",
             __func__, address, ret, physical, prot);
    if (ret == TRANSLATE_SUCCESS) {
        tlb_set_page(cs, address & TARGET_PAGE_MASK,
                     physical & TARGET_PAGE_MASK,
                     prot, mmu_idx, TARGET_PAGE_SIZE);
    } else if (ret == TRANSLATE_FAIL) {
        raise_mmu_exception(env, address, access_type);
    }
#endif

    return ret;
}

#ifndef CONFIG_USER_ONLY

/* called by qemu's softmmu to fill the qemu tlb */
void tlb_fill(CPUState *cs, target_ulong addr, MMUAccessType access_type,
        int mmu_idx, uintptr_t retaddr)
{
    int ret = riscv_cpu_handle_mmu_fault(cs, addr, access_type, mmu_idx);

    if (ret == TRANSLATE_SUCCESS)
        return;

    if(retaddr)
        cpu_restore_state(cs, retaddr);

    cpu_loop_exit(cs);
}

#endif
