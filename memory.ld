/* 小核整个镜像（code+bss+stack）放在 dtb 预留的 rtos_region [0x8FE00000,0x90000000)（2MB）。
 * 该区在 U-Boot relocaddr(0x88a32000) 之上 → 不碰 U-Boot malloc（否则 bootm 会 NULL deref）；
 * 且 dtb 标为 rtos_image → 大核 StarryOS 不分配。
 * 因 2MB 容纳不下镜像+双缓冲 YUV+JPU pool，YUV 用单缓冲（大核 cvi_yuv 用 seqlock 防撕裂）。
 * 布局（全在 rtos_region 内，640×480 YUV422=614400）：
 *   小核镜像 [0x8FE00000, ~0x8FE82000) code+bss(USB DMA_BUF 385K)+stack(64K)
 *   YUV 单缓冲 [0x8FE88000, 0x8FF1E000) 614400B (0x96000)
 *   JPU pool  [0x8FF1E000, 0x8FFFE000) 896K (0xE0000)
 *   mailbox   [0x8FFFE000, 0x8FFFE020) 32B */
MEMORY
{
    RAM (rwx) : ORIGIN = 0x8FE00000, LENGTH = 2M
}

SECTIONS
{
    . = 0x8FE00000;

    .text : {
        KEEP(*(.text.boot))
        *(.text .text.*)
    } > RAM

    .rodata : {
        . = ALIGN(8);
        *(.rodata .rodata.*)
        *(.srodata .srodata.*)
    } > RAM

    .data : {
        . = ALIGN(8);
        *(.data .data.*)
        *(.sdata .sdata.*)
        *(.got .got.*)
    } > RAM

    .bss (NOLOAD) : {
        . = ALIGN(8);
        __bss_start = .;
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(COMMON)
        . = ALIGN(8);
        __bss_end = .;
    } > RAM

    . = ALIGN(16);
    . = . + 0x10000;
    __stack_top = .;

    /DISCARD/ : {
        *(.eh_frame .eh_frame_hdr)
        *(.comment)
        *(.note .note.*)
    }
}
