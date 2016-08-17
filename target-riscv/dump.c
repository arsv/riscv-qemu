/*
 * RISC-V CPU state dumping
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
#include "qapi/error.h"
#include "qemu-common.h"

#include "cpu.h"

const char* const riscv_abi_regnames[32] = {
    "zr", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
    "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
    "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
    "s8", "s9", "sa", "sb", "t3", "t4", "t5", "t6"
};

void riscv_cpu_dump_state(CPUState *cs,
        FILE *f, fprintf_function cpu_fprintf, int flags)
{
    RISCVCPU *cpu = RISCV_CPU(cs);
    CPURISCVState *env = &cpu->env;

    int i;
    target_ulong* x = env->gpr;
    const char* const *rn = riscv_abi_regnames;

    cpu_fprintf(f, "PC 0x" TARGET_FMT_lx "\n", env->pc);
    cpu_fprintf(f, "\n");
    for(i = 0; i < 16; i++)
        cpu_fprintf(f,
            "%02i %s 0x" TARGET_FMT_lx "    "
            "%02i %s 0x" TARGET_FMT_lx "\n",
            i+0*16, rn[i+0*16], x[i+0*16],
            i+1*16, rn[i+1*16], x[i+1*16]);
}

