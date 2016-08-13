/*
 * RISC-V helper defines
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
DEF_HELPER_FLAGS_2(exception, 0, void, env, i32)

DEF_HELPER_FLAGS_2(csr, 0, void, env, i32)

DEF_HELPER_FLAGS_5(fmadd_s,  0, i64, env, i64, i64, i64, i32)
DEF_HELPER_FLAGS_5(fmsub_s,  0, i64, env, i64, i64, i64, i32)
DEF_HELPER_FLAGS_5(fnmadd_s, 0, i64, env, i64, i64, i64, i32)
DEF_HELPER_FLAGS_5(fnmsub_s, 0, i64, env, i64, i64, i64, i32)

DEF_HELPER_FLAGS_5(fmadd_d,  0, i64, env, i64, i64, i64, i32)
DEF_HELPER_FLAGS_5(fmsub_d,  0, i64, env, i64, i64, i64, i32)
DEF_HELPER_FLAGS_5(fnmadd_d, 0, i64, env, i64, i64, i64, i32)
DEF_HELPER_FLAGS_5(fnmsub_d, 0, i64, env, i64, i64, i64, i32)

DEF_HELPER_FLAGS_4(fadd_s,  0, i64, env, i64, i64, i32)
DEF_HELPER_FLAGS_4(fsub_s,  0, i64, env, i64, i64, i32)
DEF_HELPER_FLAGS_4(fmul_s,  0, i64, env, i64, i64, i32)
DEF_HELPER_FLAGS_4(fdiv_s,  0, i64, env, i64, i64, i32)
DEF_HELPER_FLAGS_3(fsqrt_s, 0, i64, env, i64, i32)
DEF_HELPER_FLAGS_3(fmin_s,  0, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmax_s,  0, i64, env, i64, i64)

DEF_HELPER_FLAGS_4(fadd_d,  0, i64, env, i64, i64, i32)
DEF_HELPER_FLAGS_4(fsub_d,  0, i64, env, i64, i64, i32)
DEF_HELPER_FLAGS_4(fmul_d,  0, i64, env, i64, i64, i32)
DEF_HELPER_FLAGS_4(fdiv_d,  0, i64, env, i64, i64, i32)
DEF_HELPER_FLAGS_3(fsqrt_d, 0, i64, env, i64, i32)
DEF_HELPER_FLAGS_3(fmin_d,  0, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmax_d,  0, i64, env, i64, i64)

DEF_HELPER_FLAGS_3(flt_s, 0, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fle_s, 0, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(feq_s, 0, i64, env, i64, i64)

DEF_HELPER_FLAGS_3(flt_d, 0, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fle_d, 0, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(feq_d, 0, i64, env, i64, i64)

DEF_HELPER_FLAGS_3(fcvt_s_d, 0, i64, env, i64, i32)
DEF_HELPER_FLAGS_3(fcvt_d_s, 0, i64, env, i64, i32)

DEF_HELPER_FLAGS_3(fcvt_w_s,  0, i64, env, i64, i32)
DEF_HELPER_FLAGS_3(fcvt_wu_s, 0, i64, env, i64, i32)
DEF_HELPER_FLAGS_3(fcvt_l_d,  0, i64, env, i64, i32)
DEF_HELPER_FLAGS_3(fcvt_lu_d, 0, i64, env, i64, i32)

DEF_HELPER_FLAGS_3(fcvt_s_w,  0, i64, env, i64, i32)
DEF_HELPER_FLAGS_3(fcvt_s_wu, 0, i64, env, i64, i32)
DEF_HELPER_FLAGS_3(fcvt_d_l,  0, i64, env, i64, i32)
DEF_HELPER_FLAGS_3(fcvt_d_lu, 0, i64, env, i64, i32)
DEF_HELPER_FLAGS_3(fcvt_d_w,  0, i64, env, i64, i32)
DEF_HELPER_FLAGS_3(fcvt_d_wu, 0, i64, env, i64, i32)

DEF_HELPER_FLAGS_1(fclass_s, 0, i64, i64)
DEF_HELPER_FLAGS_1(fclass_d, 0, i64, i64)
