.syntax unified
.cpu cortex-m4
.thumb

.equ SCB_ICSR,  0xe000ed04
.equ SYST_CSR,  0xe000e010
.equ SYST_RVR,  0xe000e014
.equ SYST_CVR,  0xe000e018

.section .isr_vector, "a", %progbits
.align 2
VectorTable:
    .word _estack
    .word Reset_Handler + 1
    .word Default_Handler + 1
    .word Default_Handler + 1
    .word Default_Handler + 1
    .word Default_Handler + 1
    .word Default_Handler + 1
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVC_Handler + 1
    .word Default_Handler + 1
    .word 0
    .word PendSV_Handler + 1
    .word SysTick_Handler + 1

.section .text.Reset_Handler, "ax", %progbits
.align 2
.global Reset_Handler
.type Reset_Handler, %function
.thumb_func
Reset_Handler:
    /* Build the basic exception frame from which PendSV starts the task. */
    ldr r0, =task_stack_top
    subs r0, #32
    movs r1, #0
    str r1, [r0, #0]
    str r1, [r0, #4]
    str r1, [r0, #8]
    str r1, [r0, #12]
    str r1, [r0, #16]
    ldr r1, =Task_Exit + 1
    str r1, [r0, #20]
    ldr r1, =Task_Entry + 1
    str r1, [r0, #24]
    ldr r1, =0x01000000
    str r1, [r0, #28]
    ldr r1, =task_initial_psp
    str r0, [r1]

    /* SVC asks PendSV to perform the first context restore. */
    svc #0
.Lreset_wait:
    b .Lreset_wait
.size Reset_Handler, . - Reset_Handler

.section .text.handlers, "ax", %progbits
.align 2
.global SVC_Handler
.type SVC_Handler, %function
.thumb_func
SVC_Handler:
    ldr r0, =svc_count
    ldr r1, [r0]
    adds r1, #1
    str r1, [r0]
    ldr r0, =SCB_ICSR
    ldr r1, =0x10000000
    str r1, [r0]
    bx lr
.size SVC_Handler, . - SVC_Handler

.global PendSV_Handler
.type PendSV_Handler, %function
.thumb_func
PendSV_Handler:
    ldr r0, =pendsv_count
    ldr r1, [r0]
    adds r1, #1
    str r1, [r0]
    ldr r0, =task_initial_psp
    ldr r0, [r0]
    msr psp, r0
    ldr r1, =0xfffffffd
    mov lr, r1
    bx lr
.size PendSV_Handler, . - PendSV_Handler

.global SysTick_Handler
.type SysTick_Handler, %function
.thumb_func
SysTick_Handler:
    ldr r0, =tick_count
    ldr r1, [r0]
    adds r1, #1
    str r1, [r0]
    mrs r1, psp
    ldr r0, =systick_frame_psp
    str r1, [r0]
    ldr r0, =SYST_CSR
    movs r1, #0
    str r1, [r0]
    bx lr
.size SysTick_Handler, . - SysTick_Handler

.type Default_Handler, %function
.thumb_func
Default_Handler:
    bkpt #0xff
.size Default_Handler, . - Default_Handler

.section .text.task, "ax", %progbits
.align 2
.global Task_Entry
.type Task_Entry, %function
.thumb_func
Task_Entry:
    ldr r0, =task_started
    ldr r1, =0x51c00001
    str r1, [r0]
    mrs r1, psp
    ldr r0, =task_entry_psp
    str r1, [r0]

    ldr r0, =SYST_RVR
    movs r1, #31
    str r1, [r0]
    ldr r0, =SYST_CVR
    movs r1, #0
    str r1, [r0]
    ldr r0, =SYST_CSR
    movs r1, #7
    str r1, [r0]

    ldr r0, =tick_count
.Lwait_for_tick:
    ldr r1, [r0]
    cmp r1, #0
    beq .Lwait_for_tick

    ldr r0, =task_resumed
    ldr r1, =0x51c00002
    str r1, [r0]
    mrs r1, psp
    ldr r0, =task_resumed_psp
    str r1, [r0]
    bkpt #0
.size Task_Entry, . - Task_Entry

.type Task_Exit, %function
.thumb_func
Task_Exit:
    bkpt #1
.size Task_Exit, . - Task_Exit

/* Keep one initialized RAM word so the fixture uses the loader's split-image
 * layout and its following NOBITS section has a representable file offset. */
.section .data.fixture, "aw", %progbits
.align 2
scheduler_fixture_data:
    .word 0x53434844

.section .bss.fixture, "aw", %nobits
.align 2
.global svc_count
svc_count:
    .space 4
.global pendsv_count
pendsv_count:
    .space 4
.global tick_count
tick_count:
    .space 4
.global task_started
task_started:
    .space 4
.global task_resumed
task_resumed:
    .space 4
.global task_initial_psp
task_initial_psp:
    .space 4
.global task_entry_psp
task_entry_psp:
    .space 4
.global systick_frame_psp
systick_frame_psp:
    .space 4
.global task_resumed_psp
task_resumed_psp:
    .space 4

.align 3
.global task_stack
task_stack:
    .space 256
.global task_stack_top
task_stack_top:
