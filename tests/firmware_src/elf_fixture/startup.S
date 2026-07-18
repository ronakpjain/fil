.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.eabi_attribute 28, 1 /* Tag_ABI_VFP_args: parameters use VFP registers. */
.thumb

.section .isr_vector, "a", %progbits
.align 2
.global VectorTable
VectorTable:
    .word 0x20020000
    .word Reset_Handler + 1

.section .text.Reset_Handler, "ax", %progbits
.align 2
.global Reset_Handler
.type Reset_Handler, %function
.thumb_func
Reset_Handler:
    ldr r0, =fixture_data
    ldr r1, =fixture_bss
    bkpt #0
.size Reset_Handler, . - Reset_Handler

.section .data.fixture, "aw", %progbits
.align 2
.global fixture_data
fixture_data:
    .word 0x12345678

.section .bss.fixture, "aw", %nobits
.align 2
.global fixture_bss
fixture_bss:
    .space 16
