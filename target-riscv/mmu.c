#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "qemu-common.h"
#include "exec/gdbstub.h"
#include "qemu/host-utils.h"
#ifndef CONFIG_USER_ONLY
#include "hw/loader.h"
#endif

static void cpu_riscv_raise_mmu_exception(RISCVCPU *cpu,
        target_ulong address, int rw, int tlb_error)
{
    CPUState *cs = CPU(cpu);
    cs->exception_index = EXCP_FAULT;
}

int riscv_cpu_handle_mmu_fault(CPUState *cs, vaddr address, int rw, int mmu_idx)
{
    RISCVCPU *cpu = RISCV_CPU(cs);

    cpu_riscv_raise_mmu_exception(cpu, address, rw, 0);

    return 1;
}
