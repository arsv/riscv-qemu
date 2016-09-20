/*
 * RISCV board definition.
 * 
 * Based on OpenRISC code by Jia Liu <proljc@gmail.com>,
 *                           Feng Gao <gf91597@gmail.com>
 * and RISC-V code by Sagar Karandikumar <sagark@eecs.berkeley.edu>
 *
 * Copyright (c) 2016 Alex Suykov
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
#include "cpubits.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "elf.h"
#include "hw/char/serial.h"
#include "net/net.h"
#include "hw/loader.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"
#include "sysemu/qtest.h"

#define KERNEL_LOAD_ADDR RISCV_DRAM_BASE

static void main_cpu_reset(void *opaque)
{
    RISCVCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
}

static void cpu_riscv_load_kernel(ram_addr_t ram_size,
                                     const char *kernel_filename,
                                     RISCVCPU *cpu)
{
    long kernel_size;
    uint64_t elf_entry;
    hwaddr entry;

    if (kernel_filename && !qtest_enabled()) {
        kernel_size = load_elf(kernel_filename, NULL, NULL,
                               &elf_entry, NULL, NULL, 1, EM_RISCV,
                               1, 0);
        entry = elf_entry;
        if (kernel_size < 0) {
            kernel_size = load_uimage(kernel_filename,
                                      &entry, NULL, NULL, NULL, NULL);
        }
        if (kernel_size < 0) {
            kernel_size = load_image_targphys(kernel_filename,
                                              KERNEL_LOAD_ADDR,
                                              ram_size);
            entry = KERNEL_LOAD_ADDR;
        }

        if (kernel_size < 0) {
            fprintf(stderr, "QEMU: couldn't load the kernel '%s'\n",
                    kernel_filename);
            exit(1);
        }
        cpu->env.pc = entry;
    }
}

static void riscv_sim_init(MachineState *machine)
{
    ram_addr_t ram_size = machine->ram_size;
    const char *cpu_model = machine->cpu_model;
    const char *kernel_filename = machine->kernel_filename;
    RISCVCPU *cpu = NULL;
    MemoryRegion *ram;
    int n;

    if (!cpu_model) {
        cpu_model = "riscv"; /* see target-riscv/model.c */
    }

    for (n = 0; n < smp_cpus; n++) {
        cpu = cpu_riscv_init(cpu_model);
        if (cpu == NULL) {
            fprintf(stderr, "Unable to find CPU definition!\n");
            exit(1);
        }
        qemu_register_reset(main_cpu_reset, cpu);
        main_cpu_reset(cpu);
    }

    ram = g_malloc(sizeof(*ram));
    memory_region_init_ram(ram, NULL, "riscv.ram", ram_size, &error_fatal);
    vmstate_register_ram_global(ram);
    memory_region_add_subregion(get_system_memory(), RISCV_DRAM_BASE, ram);

    cpu_riscv_load_kernel(ram_size, kernel_filename, cpu);
}

static void riscv_sim_machine_init(MachineClass *mc)
{
    mc->desc = "RISC-V generic board";
    mc->init = riscv_sim_init;
    mc->max_cpus = 1;
    mc->is_default = 1;
}

DEFINE_MACHINE("riscv", riscv_sim_machine_init)
