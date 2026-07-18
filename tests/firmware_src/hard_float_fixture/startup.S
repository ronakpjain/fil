.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.eabi_attribute 28, 1 /* Tag_ABI_VFP_args: parameters use VFP registers. */
.thumb

.section .isr_vector, "a", %progbits
.align 2
.global VectorTable
VectorTable:
    .word 0x20000400
    .word Reset_Handler + 1

.section .text.Reset_Handler, "ax", %progbits
.align 2
.global Reset_Handler
.type Reset_Handler, %function
.thumb_func
Reset_Handler:
    /* Enable CP10 and CP11 before entering compiler-generated hard-float code. */
    ldr r0, =0xe000ed88
    ldr r1, [r0]
    orr r1, r1, #0x00f00000
    str r1, [r0]
    dsb sy
    isb sy
    bl main
    bkpt #0
.size Reset_Handler, . - Reset_Handler
