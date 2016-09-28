/*
 * RISC-V Privileged instructions for QEMU.
 *
 * Author: Sagar Karandikar, sagark@eecs.berkeley.edu
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
#include "exception.h"
#include "exec/helper-proto.h"
#include "exec/exec-all.h"
#include "cpubits.h"

#define ENV CPURISCVState* env

#ifndef CONFIG_USER_ONLY

void HELPER(sret)(ENV)
{
    if (!(env->priv >= PRV_S))
        raise_exception(env, EXCP_ILLEGAL);

    target_ulong retpc = env->sepc;

    if (retpc & 0x3)
        raise_exception(env, EXCP_FAULT); /* XXX: should be align */

    target_ulong mstatus = env->mstatus;
    target_ulong prev_priv = get_field(mstatus, MSTATUS_SPP);
    mstatus = set_field(mstatus, MSTATUS_UIE << prev_priv,
                        get_field(mstatus, MSTATUS_SPIE));
    mstatus = set_field(mstatus, MSTATUS_SPIE, 0);
    mstatus = set_field(mstatus, MSTATUS_SPP, PRV_U);
    env->mstatus = mstatus;

    riscv_set_privilege(env, prev_priv);

    env->pc = retpc;
}

void HELPER(mret)(ENV)
{
    if (!(env->priv >= PRV_M))
        raise_exception(env, EXCP_ILLEGAL);

    target_ulong retpc = env->mepc;
    if (retpc & 0x3)
        raise_exception(env, EXCP_ILLEGAL);

    target_ulong mstatus = env->mstatus;
    target_ulong prev_priv = get_field(mstatus, MSTATUS_MPP);
    mstatus = set_field(mstatus, MSTATUS_UIE << prev_priv,
                        get_field(mstatus, MSTATUS_MPIE));
    mstatus = set_field(mstatus, MSTATUS_MPIE, 0);
    mstatus = set_field(mstatus, MSTATUS_MPP, PRV_U);
    env->mstatus = mstatus;

    riscv_set_privilege(env, prev_priv);

    env->pc = retpc;
}

void HELPER(flush)(ENV)
{
    riscv_tlb_flush(env);
}

#endif
