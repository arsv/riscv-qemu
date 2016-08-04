/*
 * RISC-V exception.
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
#include "cpu.h"
#include "exception.h"
#include "exec/exec-all.h"

/* XXX: drop _cpu_ version? Other arches seems to use this but RISC-V
   only ever calls the _env_ one. */

void QEMU_NORETURN raise_cpu_exception(RISCVCPU *cpu, uint32_t excp)
{
    CPUState *cs = CPU(cpu);

    cs->exception_index = excp;
    cpu_loop_exit_restore(cs, 0);
}

void QEMU_NORETURN raise_env_exception(CPURISCVState *env, uint32_t excp)
{
    RISCVCPU *cpu = riscv_env_get_cpu(env);
    raise_cpu_exception(cpu, excp);
}
