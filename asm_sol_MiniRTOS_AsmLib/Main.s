/*
 * Main.s
 *
 *  Created on: Dec 30, 2023
 *      Author: rozman
 */

		  .syntax unified
		  .cpu cortex-m7
		  .thumb


///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////
// Definitions section. Define all the registers and
// constants here for code readability.

// Constants

// Register Addresses
	.equ     DWT_CYCCNT,   	0xE0001004 // DWT_CYCCNT reg (RM, pp.3211)

	// RCC   base address is 0x58024400
	//   AHB4ENR register offset is 0xE0
	.equ     RCC_AHB4ENR,   0x580244E0 // RCC AHB4 peripheral clock reg

	.equ     GPIOI_BASE,   0x58022000 	// GPIOI base address)
	.equ     GPIOx_MODER,    0x00 	// GPIOx port mode register
	.equ     GPIOx_ODR,      0x14 	// GPIOx output data register
	.equ     GPIOx_BSRR,     0x18 	// GPIOx port set/reset register

    .equ     MSDELAY,       64000

//-----------------------------
// SysTick Timer definitions
//-----------------------------

	.equ     SCS_BASE,0xe000e000
	.equ     SCS_SYST_CSR,0x10// Control/Status register
	.equ     SCS_SYST_RVR,0x14// Value to countdown from
	.equ     SCS_SYST_CVR,0x18// Current value

	.equ	 SYSTICK_RELOAD_1MS,	63999  //1 msec at 64MHz ...

// Vector table offset register definition
// Important for relocated Vector table on running from RAM
	.equ VTOR,0xE000ED08


//-----------------------------
// USART3 related definitions
//-----------------------------

// RCC  base address is 0x58024400
	.equ    RCC_BASE,  0x58024400 // RCC base reg

//  APB1LENR register offset is 0xE8
	.equ    RCC_APB1LENR,  0xE8 // RCC APB1LENR peripheral clock reg

// GPIOB base address is 0x58020400
	.equ    GPIOB_BASE,  0x58020400 // GPIOB base address)

//  AFRH  register offset is 0x24
	.equ    GPIOx_AFRH,    0x24 //
	.equ    GPIO_AFRH_VALUE,    0x7700 // AF7 on PB10,11

// USART3 base address is 0x40004800
	.equ    USART3_BASE,  0x40004800 // USART3 base address)

//  CRx registers
//          CR1 register
	.equ    USART_CR1,  0x00 // CR1 register
	.equ    USART_CR1_VAL,  0b1101 // CR1 register value

	.equ    USART_CR2,  0x04 // CR2 register
	.equ    USART_CR3,  0x08 // CR3 register

//  BRR register
	.equ    USART_BRR,  0x0C // BRR register
	.equ    USART_BRR_VAL,556 // BRR register 64 000 000 / 115 200 = 555.55  = 556

//  ISR register
	.equ    USART_ISR,    0x1c // ISR register

//  Data registers
	.equ    USART_RDR,    0x24 // Receive Data Register
	.equ    USART_TDR,    0x28 // Transmit Data Register


// Comment following line when FLASH Linker Script is used
#define RAM_LinkScript

// Start of data section
 		.data

 		.align

// -------------------------------
// Mini RTOS related variables
// -------------------------------


	.equ STACKSIZE , 240 // + 16 registers of PCB

USERSTACK_Start: .space (STACKSIZE+16) * 4
USERSTACK_End:


STACK0_Start: .space STACKSIZE * 4
// PCB: ISR saves: R4-R11 (8regs), CPU saves: R0-R3, R12,      LR,        PC,         PSR
PCB0:    .word     4,5,6,7,8,9,10,11,        0,1,2,3, 12,  0xFFFFFFFF,   TASK0_Start, 0x01000000
STACK0_End:   //                                              was PSP                 Thumb mode

STACK1_Start: .space STACKSIZE * 4
PCB1:    .word     4,5,6,7,8,9,10,11,        0,1,2,3, 12,  0xFFFFFFFF,   TASK1_Start, 0x01000000
STACK1_End:   //                                              was PSP                 Thumb mode

StackPointer0:     .word     PCB0    // PSP for Task0
StackPointer1:     .word     PCB1    // PSP for Task1

ACT_TASKID: 	   .word     0         // current process ID (0,1,...)


// Start of text section
  		.text

  		.type  main, %function
  		.global main

   	   	.align
main:



#ifdef RAM_LinkScript

// Relocating Vector table to RAM (only for RAM Linker Script)
        bl RELLOC_VECTBL

#endif

		bl INIT_GPIOs

		bl INIT_USART3 // Priprava USART3 naprave

		bl ITM_Init

//		MiniRTOS_Init

        // Set new User SP that will be used for user program (MSP is reset default)
        ldr r0,=USERSTACK_End-4
        msr psp, r0

		// Set PSP as SP when stacking/unstacking user processes
		// Access special registers
		mrs  r0, CONTROL   // Read CONTROL into R0
		orr  r0, R0, #0x2  // Set SPSEL (PSP to be used in Thread mode) to select PSP
		msr  CONTROL, r0   // Write R0 into CONTROL
		isb                // Instruction Synchronization Barrier - needed

        CPSID   I          // Disable interrupts

    	bl INIT_TC_PSP     // Initialize SysTick periodic interrupt (every msec)

    	CPSIE   I          // Enable interrupts (context switches will happen)

    	b  TASK0_Start     // Go into first task (it will switch with TASK1 every msec)

__end: 	b 	__end


.global SysTick_Handler
.section .text.SysTick_Handler,"ax",%progbits
.type SysTick_Handler, %function

SysTick_Handler:
	/// STEP 1 - SAVE THE CURRENT TASK CONTEXT

	/// At this point the processor has already pushed
	/// PSR, PC, LR, R12, R3, R2, R1 and R0 onto the stack.
	/// We need to push the rest(i.e R4, R5, R6, R7, R8, R9, R10 & R11) to save the
	/// context of the current task. PSP must be used !

	/// Disable interrupts
        CPSID   I

    /// Push remaining registers R4 to R11
	//	push {r4-r11}
		mrs r3, psp       	// if 1, PSP was used
        stmfd r3!, {r4-r11} // Push onto a Full Descending Stack

		 // Note: we cannot use PUSH {r7-r9} because PUSH uses MSP
		 // in a interrupt service routine

	    ldr r0,=ACT_TASKID    // Check current Active task ID
		ldr r1,[r0]
		cmp r1,#0
		bne CONT1

		// TASK 0 current
		mov r2,#1             // Task1 is new
		str r2,[r0]

		ldr r0,=StackPointer0  // Store current PSP
		str r3,[r0]

		ldr r0,=PCB1
		ldr r1,=StackPointer1
		b   STEP2

		// TASK 1 current

CONT1:
		mov r2,#0            // Task0 is new
		str r2,[r0]

		ldr r0,=StackPointer1  // Store current PSP
		str r3,[r0]

		ldr r0,=PCB0
		ldr r1,=StackPointer0

    /// STEP 2: LOAD THE NEW PSP and TASK CONTEXT FROM ITS STACK TO THE CPU REGISTERS.

STEP2:

		ldr r3,[r1]  // Read new PSP into r3


	    /// Pop registers R4-R11
//	    pop {r4 - r11}       // pop uses MSP !!!
        ldmfd r3!, {r4-r11}  // Load from a Full Descending Stack

        msr psp, r3          // move to PSP for upcoming task

	/// Enable interrupts
	    CPSIE   I

	    BX      LR

		.global  TASK0_Start

TASK0_Start:

		bl  LED1_ON
		mov r0,#0
		mov r1,#'1'
		bl  ITM_Send

        mov r0,#'0'
        bl  SEND_UART


		mov r0,#500
		bl 	DELAY // Zakasnitev SW Delay: r0 x 1msec

		bl  LED1_OFF
		mov r0,#0
		mov r1,#'0'
		bl  ITM_Send

		mov r0,#500
		bl 	DELAY // Zakasnitev SW Delay: r0 x 1msec

		b 	TASK0_Start

		.global  TASK1_Start

TASK1_Start:

		bl  LED2_OFF
		mov r0,#1
		mov r1,#'+'
		bl  ITM_Send

        mov r0,#'1'
        bl  SEND_UART

		mov r0,#500
		bl 	DELAY // Zakasnitev SW Delay: r0 x 1msec

		bl  LED2_ON
		mov r0,#1
		mov r1,#'-'
		bl  ITM_Send

		mov r0,#500
		bl 	DELAY // Zakasnitev SW Delay: r0 x 1msec

		b 	TASK1_Start

INIT_USART3:
  		push {r0, r1, r2, lr}

		// Enable USART3 Peripheral Clock (bit 18 in APB1LENR register)
		ldr r1, =RCC_BASE          // Load peripheral clock reg base address to r1
		ldr r0, [r1,#RCC_APB1LENR]  // Read its content to r0
		orr r0, r0, #(1<<18)          // Set bit 18 to enable USART3 clock
		str r0, [r1,#RCC_APB1LENR]  // Store result in peripheral clock register

		// Enable GPIOB Peripheral Clock (bit 1 in AHB4ENR register)
		ldr r1, = RCC_AHB4ENR      // Load peripheral clock reg address to r6
		ldr r0, [r1]                // Read its content to r5
		orr r0, r0, #0b10              // Set bit 1 to enable GPIOB clock
		str r0, [r1]                // Store result in peripheral clock register

		ldr r1, =GPIOB_BASE          // Load GPIOB BASE address to r1

		// Make GPIOB Pina 10,11 as AF (bits 20:23 in MODER register)
		ldr r0, [r1,#GPIOx_MODER]  // Read GPIO_MODER content to r0
		ldr r2, =0xFF0FFFFF          // Clear mask
		and r0, r0, r2                  // Clear bits
		orr r0, #0x00A00000          // Write 10 to bits
		str r0, [r1,#GPIOx_MODER]    // Store result in GPIO MODER register

		// Make GPIOB Pina 10,11 as AF7 (bits 8:15 in AFRH register)
		ldr r0, [r1,#GPIOx_AFRH]  // Read GPIOB AFRH content to r0
		ldr r2, =0xFFFF00FF          // Clear mask
		and r0, r0, r2              // Clear bits
		orr r0, r0, #GPIO_AFRH_VALUE
		str r0, [r1,#GPIOx_AFRH]  // Store result in GPIOB AFRH register


		ldr r1, =USART3_BASE      // Load USART3 BASE address to r1

		  // Disable USART3
		mov r0, #0
		str r0, [r1,#USART_CR1]                // Store result

		// Set USART3 BaudRate
		ldr r0, =USART_BRR_VAL
		str r0, [r1,#USART_BRR]                // Store result

		// Start USART3
		mov r0, #USART_CR1_VAL
		str r0, [r1,#USART_CR1]                // Store result

  		pop {r0, r1, r2, pc}

RECV_UART:
  		push {r1, r2, lr}

      	ldr r1, =USART3_BASE
RECV_LP:
	    ldr r2, [r1, #USART_ISR]
	    tst r2, #(1 << 5)  // RXNE flag
	    beq RECV_LP

	    ldr r0, [r1, #USART_RDR]

  		pop {r1, r2, pc}


SEND_UART:
		push {r1, r2, lr}

		ldr r1, =USART3_BASE
SEND_LP:
        ldr r2, [r1, #USART_ISR]
	    tst r2, #(1 << 7)  // TXE flag
	    beq SEND_LP

	    str r0, [r1, #USART_TDR]

	    pop {r1, r2, pc}

INIT_TC_PSP:
	  	push {r0, r1, lr}
		ldr r1, =SCS_BASE

		ldr r0, =SYSTICK_RELOAD_1MS
		str r0, [r1, #SCS_SYST_RVR]

		mov r0, #0
		str r0, [r1, #SCS_SYST_CVR]

		mov r0, #0b111    // Set TickInt to 1 as well
		str r0, [r1, #SCS_SYST_CSR]

	  	pop {r0, r1, pc}

DELAY:
		push {r1,lr}

MSEC:   ldr r1,=MSDELAY
LOOP:   	subs r1,r1,#1
	        bne  LOOP

		subs r0,r0,#1
		bne  MSEC

		pop {r1,pc}

