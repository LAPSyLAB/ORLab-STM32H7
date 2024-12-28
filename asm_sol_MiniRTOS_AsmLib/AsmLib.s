/*
 * AsmLib.s
 *
 * Simple library with GPIOs, SysTick, USART assembler functions
 *
 *  Created on: Dec 30, 2023
 *      Author: rozman
 */

		  .syntax unified
		  .cpu cortex-m7
		  .thumb

// Register Addresses
// You can find the base addresses for all peripherals from Memory Map section 2.3.2
// RM0433 on page 131. Then the offsets can be found on their relevant sections.

// RCC   base address is 0x58024400
//   AHB4ENR register offset is 0xE0
	.equ     RCC_AHB4ENR,   0x580244E0 // RCC AHB4 peripheral clock reg

// GPIOA base address is 0x58020000
	.equ     GPIOA_BASE,   0x58020000 // GPIOI base address)

// GPIOI base address is 0x58022000
	.equ     GPIOI_BASE,   0x58022000 // GPIOI base address)

// GPIOJ base address is 0x58022000
	.equ     GPIOJ_BASE,   0x58022400 // GPIOJ base address)

//   MODER register offset is 0x00
	.equ     GPIOx_MODER,   0x00 // GPIOx port mode register
//   ODR   register offset is 0x14
	.equ     GPIOx_ODR,     0x14 // GPIOx output data register
//   BSSR   register offset is 0x18
	.equ     GPIOx_BSRR,     0x18 // GPIOx port set/reset register


// Values for BSRR register - pin 13: LED is on, when GPIO is off
	.equ     LED2_OFF,       0x00002000   	// Setting pin to 1 -> LED is off
	.equ     LED2_ON,   	 0x20000000   	// Setting pin to 0 -> LED is on

// Values for BSRR register - pin 2: LED is on, when GPIO is off
	.equ     LED1_OFF,       0x00000004   	// Setting pin to 1 -> LED is off
	.equ     LED1_ON,   	 0x00040000   	// Setting pin to 0 -> LED is on

// Values for BSRR register - pin 3: PA3
	.equ     PA3_ON,         0x00000008   	// Setting pin to 1
	.equ     PA3_OFF,        0x00080000   	// Setting pin to 0

// Vector table offset register definition
// Important for relocated Vector table on running from RAM
	.equ VTOR,0xE000ED08

// ITM Registers definitions
	.equ    M7_ITM_BASE, 0xE0000000


	.equ    M7_ITM_STIM0, 0x00
	.equ    M7_ITM_STIM1, 0x04

	.equ    M7_ITM_TER, 0xE00  // Enable Channels (0,1)
	.equ    M7_ITM_TCR, 0xE80  // Bit0 to 1 - enable ITM



    .global INIT_GPIOs

INIT_GPIOs:

  	push {r5, r6, lr}

	// Enable GPIOA,I,J Peripheral Clock (bit 8 in AHB4ENR register)
	ldr r6, = RCC_AHB4ENR       // Load peripheral clock reg address to r6
	ldr r5, [r6]                // Read its content to r5
	orr r5, #0x00000300         // Set bits 8 and 9 to enable GPIOI,J clock
	orr r5, #0x00000001         // Set bits 1 to enable GPIOA clock
	str r5, [r6]                // Store result in peripheral clock register

	// Make GPIOA Pin3 as output pin (bits 7:6 in MODER register)
	ldr r6, =GPIOA_BASE       // Load GPIOA BASE address to r6
	ldr r5, [r6,#GPIOx_MODER]  // Read GPIOA_MODER content to r5
	and r5, #0xFFFFFF3F          // Clear bits 7-6 for PA3
	orr r5, #0x00000040          // Write 01 to bits 7-6 for PA3
	str r5, [r6]                // Store result in GPIO MODER register

	// Make GPIOI Pin13 as output pin (bits 27:26 in MODER register)
	ldr r6, =GPIOI_BASE       // Load GPIOI BASE address to r6
	ldr r5, [r6,#GPIOx_MODER]  // Read GPIOI_MODER content to r5
	and r5, #0xF3FFFFFF          // Clear bits 27-26 for P13
	orr r5, #0x04000000          // Write 01 to bits 27-26 for P13
	str r5, [r6]                // Store result in GPIO MODER register

	// Make GPIOJ Pin2 as output pin (bits 5:4 in MODER register)
	ldr r6, =GPIOJ_BASE       // Load GPIOJ BASE address to r6
	ldr r5, [r6,#GPIOx_MODER]  // Read GPIOJ_MODER content to r5
	and r5, #0xFFFFFFCF          // Clear bits 5-4 for P2
	orr r5, #0x00000010          // Write 01 to bits 5-4 for PJ2
	str r5, [r6]                // Store result in GPIO MODER register

  	pop {r5, r6, pc}


    .global LED1_ON
LED1_ON:
		push {r5,r6,lr}

		ldr r6, =GPIOJ_BASE       // Load GPIOI BASE address to r6
		// Led On
//		mov    r5, #LED1_ON
		mov    r5, #0x00040000
 		str    r5, [r6,#GPIOx_BSRR] // Write to BSRR register

		pop {r5,r6,pc}

    .global LED1_OFF
LED1_OFF:
		push {r5,r6,lr}

		ldr r6, =GPIOJ_BASE       // Load GPIOI BASE address to r6
		// Led Off
//		mov    r5, #LED1_OFF
		mov    r5, #0x00000004
 		str    r5, [r6,#GPIOx_BSRR] // Write to BSRR register

		pop {r5,r6,pc}


    .global LED2_ON
LED2_ON:
		push {r5,r6,lr}

		ldr r6, =GPIOI_BASE       // Load GPIOI BASE address to r6
		// Led On
//		mov    r5, #LED2_ON
		mov    r5, #0x20000000
 		str    r5, [r6,#GPIOx_BSRR] // Write to BSRR register

		pop {r5,r6,pc}


    .global LED2_OFF
LED2_OFF:
		push {r5,r6,lr}

		ldr r6, =GPIOI_BASE       // Load GPIOI BASE address to r6
		// Led Off
//		mov    r5, #LED2_OFF
		mov    r5, #0x00002000
 		str    r5, [r6,#GPIOx_BSRR] // Write to BSRR register

		pop {r5,r6,pc}

    .global PA3_ON
PA3_ON:
		push {r5,r6,lr}

		ldr r6, =GPIOA_BASE       // Load GPIOI BASE address to r6
		// Led On
//		mov    r5, #PA3_ON
 		mov    r5, #0x00000008
 		str    r5, [r6,#GPIOx_BSRR] // Write to BSRR register

		pop {r5,r6,pc}


    .global PA3_OFF
PA3_OFF:
		push {r5,r6,lr}

		ldr r6, =GPIOA_BASE       // Load GPIOI BASE address to r6
		// Led Off
//		mov    r5, #PA3_OFF
		mov    r5, #0x00080000
 		str    r5, [r6,#GPIOx_BSRR] // Write to BSRR register

		pop {r5,r6,pc}



    .global RELLOC_VECTBL

RELLOC_VECTBL:

  	push {r0, r1, lr}

    ldr r1, =VTOR // Set Vector table addr. to 0x24000000
	ldr r0, =0x24000000
	str r0, [r1]

  	pop {r0, r1, pc}

    .global ITM_Init

ITM_Init:

  	push {r0, r1, lr}

    ldr r0, =M7_ITM_BASE // Base address
    mov r1, #0b11
    str r1,[r0,#M7_ITM_TER]

    mov r1, #1
    str r1,[r0,#M7_ITM_TCR]

  	pop {r0, r1, pc}

    .global ITM_Send

ITM_Send:

  	push {r2, lr}

	ldr r2, =M7_ITM_BASE // Base address

    cmp r0,#0 // Channel 0 ?
    bne Ch1
    str r1,[r2]
    b   contx
Ch1:
	str r1,[r2,#4]

contx:

  	pop {r2, pc}

