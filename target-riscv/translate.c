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

typedef struct DisasContext {
    TranslationBlock *tb;
    target_ulong pc;
} DisasContext;

static TCGv_env cpu_env;
#include "exec/gen-icount.h"

void restore_state_to_opc(CPURISCVState *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->pc = data[0];
}

static void gen_exception(DisasContext *dc, unsigned int excp)
{
    printf("%s\n", __FUNCTION__);
    TCGv_i32 tmp = tcg_const_i32(excp);
    gen_helper_exception(cpu_env, tmp);
    tcg_temp_free_i32(tmp);
}

void gen_intermediate_code(CPURISCVState *env, struct TranslationBlock *tb)
{
    struct DisasContext ctx, *dc = &ctx;
    int num_insns = 0;

    dc->tb = tb;
    dc->pc = tb->pc;

    gen_tb_start(tb);
    gen_exception(dc, EXCP_ILLEGAL);
    gen_tb_end(tb, num_insns);

    tb->size = 4;
    tb->icount = num_insns;
}

void riscv_translate_init(void)
{
    static bool initialized = 0;

    if(!tcg_enabled())
	    return;
    if(initialized)
	    return;
    initialized = 1;

    printf("%s\n", __FUNCTION__);

    cpu_env = tcg_global_reg_new_ptr(TCG_AREG0, "env");
    tcg_ctx.tcg_env = cpu_env;

    // stub
}
