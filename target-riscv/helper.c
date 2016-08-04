#include "qemu/osdep.h"
#include "exception.h"
#include "exec/helper-proto.h"
#include "exec/exec-all.h"

void HELPER(exception)(CPURISCVState *env, uint32_t excp)
{
    raise_env_exception(env, excp);
}
