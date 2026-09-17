// arch/asm.S — 架构层全部全局汇编(经 asm.rs 的 global_asm!(include_str!) 引入)。
//
// 内容:
//   _start       启动入口(U-Boot cvi_reset_c906l 复位到 rtos_region 基址)。
//                .text.boot 段由链接脚本 KEEP 在镜像头部;清 bss 后跳 rust_main,
//                返回则 wfi 挂死(裸机主循环永不返回)。
//   _trap_entry  M-mode trap 入口:保存全部寄存器 → a0=mcause/a1=sp 调
//                rust_trap_handler → 恢复 → mret。
//
// 被 call 的符号均为 #[no_mangle] extern "C",直接按名引用,无需模板参数。

    .section .text.boot, "ax"
    .global _start
    .type _start, @function
_start:
    csrw    mie, zero
    la      sp, __stack_top
    la      t0, __bss_start
    la      t1, __bss_end
1:  bgeu    t0, t1, 2f
    sd      zero, 0(t0)
    addi    t0, t0, 8
    j       1b
2:  call    rust_main
3:  wfi
    j       3b

    .section .text.trap, "ax"
    .align  2
    .global _trap_entry
    .type   _trap_entry, @function
_trap_entry:
    addi    sp, sp, -256
    sd      ra,   0(sp)
    sd      gp,   8(sp)
    sd      tp,  16(sp)
    sd      t0,  24(sp)
    sd      t1,  32(sp)
    sd      t2,  40(sp)
    sd      s0,  48(sp)
    sd      s1,  56(sp)
    sd      a0,  64(sp)
    sd      a1,  72(sp)
    sd      a2,  80(sp)
    sd      a3,  88(sp)
    sd      a4,  96(sp)
    sd      a5, 104(sp)
    sd      a6, 112(sp)
    sd      a7, 120(sp)
    sd      s2, 128(sp)
    sd      s3, 136(sp)
    sd      s4, 144(sp)
    sd      s5, 152(sp)
    sd      s6, 160(sp)
    sd      s7, 168(sp)
    sd      s8, 176(sp)
    sd      s9, 184(sp)
    sd      s10,192(sp)
    sd      s11,200(sp)
    sd      t3, 208(sp)
    sd      t4, 216(sp)
    sd      t5, 224(sp)
    sd      t6, 232(sp)
    csrr    t0, mepc
    sd      t0, 240(sp)
    csrr    t0, mstatus
    sd      t0, 248(sp)
    csrr    a0, mcause
    mv      a1, sp
    call    rust_trap_handler
    mv      sp, a0
    ld      t0, 248(sp)
    csrw    mstatus, t0
    ld      t0, 240(sp)
    csrw    mepc, t0
    ld      ra,   0(sp)
    ld      gp,   8(sp)
    ld      tp,  16(sp)
    ld      t0,  24(sp)
    ld      t1,  32(sp)
    ld      t2,  40(sp)
    ld      s0,  48(sp)
    ld      s1,  56(sp)
    ld      a0,  64(sp)
    ld      a1,  72(sp)
    ld      a2,  80(sp)
    ld      a3,  88(sp)
    ld      a4,  96(sp)
    ld      a5, 104(sp)
    ld      a6, 112(sp)
    ld      a7, 120(sp)
    ld      s2, 128(sp)
    ld      s3, 136(sp)
    ld      s4, 144(sp)
    ld      s5, 152(sp)
    ld      s6, 160(sp)
    ld      s7, 168(sp)
    ld      s8, 176(sp)
    ld      s9, 184(sp)
    ld      s10,192(sp)
    ld      s11,200(sp)
    ld      t3, 208(sp)
    ld      t4, 216(sp)
    ld      t5, 224(sp)
    ld      t6, 232(sp)
    addi    sp, sp, 256
    mret
