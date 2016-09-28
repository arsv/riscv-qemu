/*
 * RISC-V virtual CPU, core QOM routines
 *
 * Author: Sagar Karandikar <sagark@eecs.berkeley.edu>
 *         Alex Suykov <alex.suykov@gmail.com>
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
#include "translate.h"

/* Top-down file, the main object definition is at the bottom. */

/* Which of TYPE_RISCV_CPU* classes to use for a given cpu_model?
   There are no subclasses atm, so it's always TYPE_RISCV_CPU.
   Called by cpu_riscv_init() via cpu_generic_init(). */

static ObjectClass *riscv_cpu_class_by_name(const char *cpu_model)
{
    if(cpu_model == NULL)
        return NULL;

    if(!strcmp(cpu_model, "any"))
        return object_class_by_name(TYPE_RISCV_CPU);

    return NULL;
}

/* Proper (non-static) object methods follow */

static void riscv_cpu_init(Object *obj)
{
    CPUState *cs = CPU(obj);
    RISCVCPU *cpu = RISCV_CPU(obj);

    cs->env_ptr = &cpu->env;
    cpu_exec_init(cs, &error_abort);

    riscv_translate_init();
}

static void riscv_cpu_realize(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    RISCVCPUClass *occ = RISCV_CPU_GET_CLASS(dev);

    qemu_init_vcpu(cs);

    occ->parent_realize(dev, errp);
}

static void riscv_cpu_reset(CPUState *s)
{
    RISCVCPU *cpu = RISCV_CPU(s);
    CPURISCVState *env = &cpu->env;

    memset(env, 0, sizeof(*env));

    /* ISA spec 7.3: whenever NaN is raised, it is the canonical NaN. */
    env->fpstatus.default_nan_mode = true;

    /* No point in setting env->pc for linux-user mode, it's going
       to be set anyway by the elf loader. */

    s->exception_index = -1;
}

static void riscv_cpu_set_pc(CPUState *cs, vaddr value)
{
    RISCVCPU *cpu = RISCV_CPU(cs);

    cpu->env.pc = value;
}

/* QOM stuff, class registration. */

static void riscv_cpu_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    RISCVCPUClass *occ = RISCV_CPU_CLASS(oc);
    CPUClass *cc = CPU_CLASS(occ);

    occ->parent_realize = dc->realize;
    dc->realize = riscv_cpu_realize;

    occ->parent_reset = cc->reset;
    cc->reset = riscv_cpu_reset;

    cc->class_by_name = riscv_cpu_class_by_name;
    cc->has_work = riscv_cpu_has_work;
    cc->do_interrupt = riscv_cpu_do_interrupt;
    cc->cpu_exec_interrupt = riscv_cpu_exec_interrupt;
    cc->dump_state = riscv_cpu_dump_state;
    cc->set_pc = riscv_cpu_set_pc;

    cc->gdb_num_core_regs = 32 + 1 + 32; /* gpr, pc, fpr */
    cc->gdb_read_register = riscv_cpu_gdb_read_register;
    cc->gdb_write_register = riscv_cpu_gdb_write_register;

    cc->handle_mmu_fault = riscv_cpu_handle_mmu_fault; /* linux-user only */

    /* Reason: riscv_cpu_init() calls cpu_exec_init(), which saves
       the object in cpus -> dangling pointer after final object_unref(). */
    dc->cannot_destroy_with_object_finalize_yet = true;
}

static const TypeInfo riscv_cpu_type_info = {
    .name = TYPE_RISCV_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(RISCVCPU),
    .instance_init = riscv_cpu_init,
    .class_size = sizeof(RISCVCPUClass),
    .class_init = riscv_cpu_class_init,
};

static void riscv_cpu_register_types(void)
{
    type_register(&riscv_cpu_type_info);
}

type_init(riscv_cpu_register_types)
