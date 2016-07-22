#ifndef TARGET_RISCV_EXCEPTION_H
#define TARGET_RISCV_EXCEPTION_H

#include "cpu.h"
#include "qemu-common.h"

void QEMU_NORETURN raise_exception(RISCVCPU *cpu, uint32_t excp);

#endif /* TARGET_RISCV_EXCEPTION_H */
