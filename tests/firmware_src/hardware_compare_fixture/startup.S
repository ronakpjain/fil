.syntax unified
.cpu cortex-m4
.thumb

/*
 * Deterministic hardware-comparison probe.
 *
 * Every compared general-purpose register and result byte is initialized before
 * BKPT. The final CMP also fixes the APSR N/Z/C/V flags, while the vector table
 * fixes SP. This avoids treating architecturally undefined reset values as an
 * emulator mismatch.
 */

.section .isr_vector, "a", %progbits
.align 2
.global VectorTable
VectorTable:
    .word 0x20001000
    .word Reset_Handler + 1

.section .text.Reset_Handler, "ax", %progbits
.align 2
.global Reset_Handler
.type Reset_Handler, %function
.thumb_func
Reset_Handler:
    /* Materialize a byte-for-byte RAM result block. */
    ldr r0, =probe_memory
    ldr r1, =0x11223344
    str r1, [r0, #0]
    ldr r1, =0xa5a55a5a
    str r1, [r0, #4]
    ldr r1, =0x0badc0de
    str r1, [r0, #8]
    ldr r1, =0xfeedbeef
    str r1, [r0, #12]

    /* Initialize every core register whose reset value would otherwise vary. */
    movs r0, #0x10
    movs r1, #0x11
    movs r2, #0x12
    movs r3, #0x13
    movs r4, #0x14
    movs r5, #0x15
    movs r6, #0x16
    movs r7, #0x17
    mov.w r8, #0x18
    mov.w r9, #0x19
    mov.w r10, #0x1a
    mov.w r11, #0x1b
    mov.w r12, #0x1c
    mov.w lr, #0x1e

    /* N=0, Z=1, C=1, V=0 immediately before the debug boundary. */
    cmp r0, r0
    bkpt #0
.size Reset_Handler, . - Reset_Handler

.section .data.probe, "aw", %progbits
.align 2
.global probe_memory
.type probe_memory, %object
probe_memory:
    .word 0, 0, 0, 0
.size probe_memory, . - probe_memory
