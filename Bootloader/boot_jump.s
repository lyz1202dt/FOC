.syntax unified
.cpu cortex-m4
.thumb

.section .text.BootJumpToApp,"ax",%progbits
.global BootJumpToApp
.type BootJumpToApp, %function
.thumb_func
BootJumpToApp:
    /* No C code or stack accesses after installing the application's MSP. */
    ldr r1, [r0]
    ldr r2, [r0, #4]
    movs r3, #0
    msr control, r3
    msr basepri, r3
    msr faultmask, r3
    isb
    msr msp, r1
    cpsie i
    bx r2
.size BootJumpToApp, .-BootJumpToApp
