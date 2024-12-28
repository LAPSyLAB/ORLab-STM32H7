/*
 * Main.s
 *
 *  Created on: Aug 24, 2022
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

	// Values for BSSR register - pin 13: LED is on, when GPIO is off
	.equ     LEDs_OFF,      0x00002000   // Setting pin to 1 -> LED is off
	.equ     LEDs_ON,    	0x20000000   // Setting pin to 0 -> LED is on

    .equ     MSDELAY,       64000


    // SysTick Timer definitions
	.equ     SCS_BASE,0xe000e000
	.equ     SCS_SYST_CSR,0x10// Control/Status register
	.equ     SCS_SYST_RVR,0x14// Value to countdown from
	.equ     SCS_SYST_CVR,0x18// Current value

	.equ	 SYSTICK_RELOAD_1MS,	63999  //1 msec at 64MHz ...

// Vector table offset register definition
// Important for relocated Vector table on running from RAM
	.equ VTOR,0xE000ED08




// Start of data section
 		.data

 		.align

LEDSTAT: .word 0 // LED state
MSECCNT: .word 0 //MSecs counter for SysTick_Handler
MSECMAX: .word 500 //MSecs interval for SysTick_Handler


// Start of text section
  		.text

  		.type  main, %function
  		.global main

   	   	.align
main:
        bl INIT_IO

        ldr r1, =VTOR // Set Vector table addr. to 0x24000000
		ldr r0, =0x24000000
		str r0, [r1]

		bl INIT_TC_PSP // Priprava SysTick časovnika s prek

/*
loop:

        bl LED_ON

        mov r0,#500
//        bl  DELAY
        bl DELAYTC

        bl LED_OFF

        mov r0,#500
//        bl  DELAY
        bl DELAYTC


        b  loop*/

__end: 	b 	__end


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

.global SysTick_Handler
.section .text.SysTick_Handler,"ax",%progbits
.type SysTick_Handler, %function

SysTick_Handler:

		push {r3, r4, r5, r6, lr}

		ldr r3,=MSECMAX // Load MAX value
		ldr r5,[r3]

		ldr r3,=MSECCNT // Load MSecs Counter value
		ldr r4,[r3]
		add r4,r4,#1 // Increment (+1)
		str r4,[r3]

		cmp r4,r5 // End of interval ?
		blo RET

		// Reset counter state and check LED
		mov r4,#0
		str r4,[r3]

		ldr r3,=LEDSTAT
		ldr r4,[r3]
		cmp r4,#0
		beq LON

		mov r5, #LEDs_OFF
		mov r4,#0
		str r4,[r3]
		b CONT

LON: 	mov r5, #LEDs_ON
		mov r4,#0xff
		str r4,[r3]

CONT:
		// Set GPIOI Pins through BSSR register
		ldr r6, =GPIOI_BASE // Load GPIOD BASE address to r6
		str r5, [r6,#GPIOx_BSRR] // Write to BSRR register

RET: 	pop {r3, r4, r5, r6, pc }



INIT_TC:
	  	push {r0, r1, lr}
		ldr r1, =SCS_BASE

		ldr r0, =SYSTICK_RELOAD_1MS
		str r0, [r1, #SCS_SYST_RVR]

		mov r0, #0
		str r0, [r1, #SCS_SYST_CVR]

		mov r0, #0b101
		str r0, [r1, #SCS_SYST_CSR]

	  	pop {r0, r1, pc}


INIT_IO:
		push {r5,r6,lr}


		// 1. RCC_AHB4ENR(Peripheral Clock Register): b8=1 .. Port I Enable
		// Enable GPIOI Peripheral Clock (bit 8 in AHB4ENR register)
		ldr r6, = RCC_AHB4ENR       // Load peripheral clock reg address to r6
		ldr r5, [r6]                // Read its content to r5
		orr r5, #0x00000100          // Set bit 8 to enable GPIOI clock
		str r5, [r6]                // Store result in peripheral clock register

		// 2. MODER (Mode Register): 01: General purpose output mode
		// Make GPIOI Pin13 as output pin (bits 27:26 in MODER register)
		ldr r6, =GPIOI_BASE       // Load GPIOI BASE address to r6
		ldr r5, [r6,#GPIOx_MODER]  // Read GPIOI_MODER content to r5
		and r5, #0xF3FFFFFF          // Clear bits 27-26 for P13
		orr r5, #0x04000000          // Write 01 to bits 27-26 for P13
		str r5, [r6]                // Store result in GPIO MODER register

		pop {r5,r6,pc}


// Delay with internal timer based loop approx. r0 x ms
DELAYTC:
	    push {r1, r2, lr}
	    ldr r1, =SCS_BASE

LOOPTC:		ldr r2, [r1, #SCS_SYST_CSR]
			tst r2, #0x10000   // COUNT_FLAG=1?
			beq LOOPTC

      	subs r0, r0, #1
     	bne LOOPTC

    	pop {r1, r2, pc}

LED_ON:
		push {r5,r6,lr}

		ldr r6, =GPIOI_BASE       // Load GPIOI BASE address to r6
		// Led On
		mov    r5, #LEDs_ON
 		str    r5, [r6,#GPIOx_BSRR] // Write to BSRR register

		pop {r5,r6,pc}


LED_OFF:
		push {r5,r6,lr}

		ldr r6, =GPIOI_BASE       // Load GPIOI BASE address to r6
		// Led Off
		mov    r5, #LEDs_OFF
 		str    r5, [r6,#GPIOx_BSRR] // Write to BSRR register

		pop {r5,r6,pc}


DELAY:
		push {r1,lr}

MSEC:   ldr r1,=MSDELAY
LOOP:   	subs r1,r1,#1
	        bne  LOOP

		subs r0,r0,#1
		bne  MSEC

		pop {r1,pc}

