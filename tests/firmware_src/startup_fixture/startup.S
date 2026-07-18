.syntax unified
.cpu cortex-m4
.thumb

.section .isr_vector, "a", %progbits
.align 2
VectorTable:
    .word 0x20000100
    .word Reset_Handler + 1

.section .text.Reset_Handler, "ax", %progbits
.align 2
.global Reset_Handler
.type Reset_Handler, %function
.thumb_func
Reset_Handler:
    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata
.Lcopy_data:
    cmp r1, r2
    bcs .Lzero_setup
    ldr r3, [r0]
    str r3, [r1]
    adds r0, #4
    adds r1, #4
    b .Lcopy_data

.Lzero_setup:
    ldr r1, =_sbss
    ldr r2, =_ebss
    movs r3, #0
.Lzero_bss:
    cmp r1, r2
    bcs main
    str r3, [r1]
    adds r1, #4
    b .Lzero_bss
.size Reset_Handler, . - Reset_Handler

.type main, %function
.global main
.thumb_func
main:
    ldr r0, =startup_sentinel
    movw r1, #0xbeef
    movt r1, #0xfeed
    str r1, [r0]
    bkpt #0
.size main, . - main

.section .data.fixture, "aw", %progbits
.align 2
.global startup_data
startup_data:
    .word 0x12345678
    .word 0xa5a55a5a
    .word 0x0badc0de

.section .bss.fixture, "aw", %nobits
.align 2
.global startup_bss
startup_bss:
    .space 16
.global startup_sentinel
startup_sentinel:
    .space 4
