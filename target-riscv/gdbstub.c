/*
 * RISC-V gdb server stub
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
            env->fpr[n-33] = ldl_p(mem_buf);
            return sizeof(target_long); /* target_float actually */
        default:
            return 0;
    }
}
