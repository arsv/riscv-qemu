#include "qemu/osdep.h"
#include "qemu-common.h"
#include "cpu.h"
#include "exec/gdbstub.h"

int riscv_cpu_gdb_read_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;

    switch(n) {
        case 0 ... 31:
            return gdb_get_regl(mem_buf, env->gpr[n]);
        case 32:
            return gdb_get_regl(mem_buf, env->pc);
        case 33 ... 64:
            return gdb_get_regl(mem_buf, env->fpr[n-33]);
        default:
            return 0;

    }
}

int riscv_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;

    switch(n) {
        case 0 ... 31:
            env->gpr[n] = ldl_p(mem_buf);
            return sizeof(target_long);
        case 32:
            env->pc = ldl_p(mem_buf);
            return sizeof(target_long);
        case 33 ... 64:
            env->fpr[n] = ldl_p(mem_buf);
            return sizeof(target_long); /* target_float actually */
        default:
            return 0;
    }
}
