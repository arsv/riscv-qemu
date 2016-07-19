#include "qemu/osdep.h"
#include "cpu.h"
#include "translate.h"
#include "qemu-common.h"

#include "tcg-op.h"
#include "qemu/log.h"
#include "qemu/bitops.h"
#include "exec/cpu_ldst.h"

#include "exec/helper-proto.h"
#include "exec/helper-gen.h"

#include "trace-tcg.h"
#include "exec/log.h"

static TCGv_env cpu_env;
#include "exec/gen-icount.h"

void restore_state_to_opc(CPURISCVState *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->pc = data[0];
}

void gen_intermediate_code(CPURISCVState *env, struct TranslationBlock *tb)
{
    int num_insns = 0;

    gen_tb_start(tb);

    gen_tb_end(tb, num_insns);

    tb->size = 0;
    tb->icount = num_insns;
}

void riscv_translate_init(void)
{

}
