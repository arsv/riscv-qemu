#ifndef __RISCV_CPUBITS_H__
#define __RISCV_CPUBITS_H__

/* Address space layout */
#define RISCV_MMIO_BASE    0x40000000
#define RISCV_DRAM_BASE    0x80000000

/* env->priv */
#define PRV_U 0
#define PRV_S 1
#define PRV_H 2
#define PRV_M 3

#define VM_MBARE 0
#define VM_MBB   1
#define VM_MBBID 2
#define VM_SV32  8
#define VM_SV39  9
#define VM_SV48  10

/* env->mstatus */
#define MSTATUS_UIE        0x00000001
#define MSTATUS_SIE        0x00000002
#define MSTATUS_HIE        0x00000004
#define MSTATUS_MIE        0x00000008
#define MSTATUS_UPIE       0x00000010
#define MSTATUS_SPIE       0x00000020
#define MSTATUS_HPIE       0x00000040
#define MSTATUS_MPIE       0x00000080
#define MSTATUS_SPP        0x00000100
#define MSTATUS_HPP        0x00000600
#define MSTATUS_MPP        0x00001800
#define MSTATUS_FS         0x00006000
#define MSTATUS_XS         0x00018000
#define MSTATUS_MPRV       0x00020000
#define MSTATUS_PUM        0x00040000
#define MSTATUS_MXR        0x00080000
#define MSTATUS_VM         0x1F000000

#define MSTATUS32_SD       0x80000000
#define MSTATUS64_SD       0x8000000000000000

#define PGSHIFT 12

/* page table entry (PTE) fields */
#define PTE_V     0x001 /* Valid */
#define PTE_R     0x002 /* Read */
#define PTE_W     0x004 /* Write */
#define PTE_X     0x008 /* Execute */
#define PTE_U     0x010 /* User */
#define PTE_G     0x020 /* Global */
#define PTE_A     0x040 /* Accessed */
#define PTE_D     0x080 /* Dirty */
#define PTE_SOFT  0x300 /* Reserved for Software */

#define PTE_PPN_SHIFT 10

#define PTE_TABLE(PTE) (((PTE) & (PTE_V | PTE_R | PTE_W | PTE_X)) == PTE_V)
/* end Spike decode.h, encoding.h section */

#define IRQ_S_SOFT   1
#define IRQ_H_SOFT   2
#define IRQ_M_SOFT   3
#define IRQ_S_TIMER  5
#define IRQ_H_TIMER  6
#define IRQ_M_TIMER  7
#define IRQ_S_EXT    9
#define IRQ_H_EXT    10
#define IRQ_M_EXT    11
#define IRQ_COP      12
#define IRQ_HOST     13

#define RISCV_EXCP_INST_ADDR_MIS           0x0
#define RISCV_EXCP_INST_ACCESS_FAULT       0x1
#define RISCV_EXCP_ILLEGAL_INST            0x2
#define RISCV_EXCP_BREAKPOINT              0x3
#define RISCV_EXCP_LOAD_ADDR_MIS           0x4
#define RISCV_EXCP_LOAD_ACCESS_FAULT       0x5
#define RISCV_EXCP_STORE_AMO_ADDR_MIS      0x6
#define RISCV_EXCP_STORE_AMO_ACCESS_FAULT  0x7
#define RISCV_EXCP_U_ECALL                 0x8
#define RISCV_EXCP_S_ECALL                 0x9
#define RISCV_EXCP_H_ECALL                 0xa
#define RISCV_EXCP_M_ECALL                 0xb

#define get_field(reg, mask) (((reg) & \
                 (target_ulong)(mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(target_ulong)(mask)) | \
                 (((target_ulong)(val) * ((mask) & ~((mask) << 1))) & \
                 (target_ulong)(mask)))

#endif
