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

    /* No point in setting env->pc for linux-user mode, it's going
       to be set anyway by the elf loader. */

    /* XXX: why is this necessary? */
    s->exception_index = EXCP_NONE;
}

static bool riscv_cpu_has_work(CPUState *cs)
{
    return 0;
}

static void riscv_cpu_do_interrupt(CPUState *cs)
{
    /* stub */
    printf("%s\n", __FUNCTION__);
}

static bool riscv_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    /* stub */
    printf("%s %i\n", __FUNCTION__, interrupt_request);
    return 0;
}

static void riscv_cpu_set_pc(CPUState *cs, vaddr value)
{
    RISCVCPU *cpu = RISCV_CPU(cs);

    printf("%s %li\n", __FUNCTION__, value);

    cpu->env.pc = value;
}

static int riscv_cpu_handle_mmu_fault(CPUState *cs,
		vaddr address, int rw, int mmu_idx)
{
    cs->exception_index = EXCP_FAULT;

    return 1;
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
