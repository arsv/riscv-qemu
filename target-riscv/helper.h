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

DEF_HELPER_FLAGS_4(FMADDS,  0, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(FMSUBS,  0, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(FNMADDS, 0, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(FNMSUBS, 0, i64, env, i64, i64, i64)

DEF_HELPER_FLAGS_4(FMADDD,  0, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(FMSUBD,  0, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(FNMADDD, 0, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(FNMSUBD, 0, i64, env, i64, i64, i64)
