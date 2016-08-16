/*
 * RISC-V CPU model init
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

/* Handler for qemu-riscv -cpu help.
   It should list values for cpu_model below, but at this point model
   selection is not supported. There is only one cpu class that aims
   to implement RV64g. */

void cpu_riscv_list(FILE *f, fprintf_function cpu_fprintf)
{
    (*cpu_fprintf)(f, "No specific CPU models are supported.\n");
    (*cpu_fprintf)(f, "Use \"any\" or just skip it altogether.\n");
}

/* This gets cpu model and returns initialized CPU object.
   Model is whatever was passed via -cpu, or .cpu_model from board spec,
   or the default value "any" set in linux-user/main.c.
 
   Note cpu_generic_init does not do model selection itself.
   Instead, it picks the parent CPU class (TYPE_RISCV_CPU) and calls
   its class_by_name(cpu_model) to get proper subclass name for given
   model, which is then instantiated. */

RISCVCPU *cpu_riscv_init(const char *cpu_model)
{
    return RISCV_CPU(cpu_generic_init(TYPE_RISCV_CPU, cpu_model));
}
