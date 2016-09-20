#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"

#include "cpu.h"
#include "exception.h"

#ifndef CONFIG_USER_ONLY

void tlb_fill(CPUState *cs, target_ulong addr, MMUAccessType at,
        int mmu_idx, uintptr_t retaddr)
{
    /* stub: unconditional MMU fault */

    if(retaddr)
        cpu_restore_state(cs, retaddr);

    cpu_loop_exit(cs);
}

#endif
