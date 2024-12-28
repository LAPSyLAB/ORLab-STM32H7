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

// USART3 related definitions

// RCC   base address is 0x58024400
	.equ     RCC_BASE,   0x58024400 // RCC base reg

//   APB1LENR register offset is 0xE8
	.equ     RCC_APB1LENR,   0xE8 // RCC APB1LENR peripheral clock reg

// GPIOB base address is 0x58020400
	.equ     GPIOB_BASE,   0x58020400 // GPIOB base address)

//   AFRH  register offset is 0x24
	.equ     GPIOx_AFRH,     	0x24 		//
	.equ     GPIO_AFRH_VALUE,     0x7700 		// AF7 on PB10,11

// USART3 base address is 0x40004800
	.equ     USART3_BASE,   0x40004800 // USART3 base address)

//   CRx registers
	.equ     USART_CR1,   0x00 // CR1 register
	.equ     USART_CR1_VAL,   0b1101 // CR1 register value

	.equ     USART_CR2,   0x04 // CR2 register
	.equ     USART_CR3,   0x08 // CR3 register

//   BRR register
	.equ     USART_BRR,   0x0C // BRR register
	.equ     USART_BRR_VAL,556 // BRR register 64 000 000 / 115 200 = 555.55  = 556

//   ISR register
	.equ     USART_ISR,    0x1c // ISR register

//   Data registers
	.equ     USART_RDR,    0x24 // Receive Data Register
	.equ     USART_TDR,    0x28 // Transmit Data Register



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

	    bl INIT_USART3 // Priprava USART3 naprave


		mov r5,#'a'-1

loop:
	    add r5,r5,#1    // transmit chars from 'a' to 'z'
	    cmp r5,#'z'
	    bls cont
	    mov r5,#'a'

cont:   mov r0,r5
	    bl	SEND_UART

		mov r0,#500
		bl DELAY	 	// Zakasnitev SW Delay: r0 x 1msec

//   	 	bl	RECV_UART   // will return only if character is received

		b loop          // skok na vrstico loop:

__end: 	b 	__end

INIT_USART3:
	  	push {r0, r1, r2, lr}



		// Enable USART3 Peripheral Clock (bit 18 in APB1LENR register)
		ldr r1, =RCC_BASE          	 // Load peripheral clock reg base address to r1
		ldr r0, [r1,#RCC_APB1LENR]   // Read its content to r0
		orr r0, r0, #(1<<18)          	 // Set bit 18 to enable USART3 clock
		str r0, [r1,#RCC_APB1LENR]   // Store result in peripheral clock register

		// Enable GPIOB Peripheral Clock (bit 1 in AHB4ENR register)
		ldr r1, = RCC_AHB4ENR       // Load peripheral clock reg address to r6
		ldr r0, [r1]                // Read its content to r5
		orr r0, r0, #0b10               // Set bit 1 to enable GPIOB clock
		str r0, [r1]                // Store result in peripheral clock register

		ldr r1, =GPIOB_BASE          // Load GPIOB BASE address to r1

		// Make GPIOB Pina 10,11 as AF (bits 20:23 in MODER register)
		ldr r0, [r1,#GPIOx_MODER]  // Read GPIO_MODER content to r0
		ldr r2, =0xFF0FFFFF          // Clear mask
		and r0, r0, r2                   // Clear bits
		orr r0, #0x00A00000          // Write 10 to bits
		str r0, [r1,#GPIOx_MODER]     // Store result in GPIO MODER register

		// Make GPIOB Pina 10,11 as AF7 (bits 8:15 in AFRH register)
		ldr r0, [r1,#GPIOx_AFRH]  // Read GPIOB AFRH content to r0
		ldr r2, =0xFFFF00FF          // Clear mask
		and r0, r0, r2               // Clear bits
		orr r0, r0, #GPIO_AFRH_VALUE
		str r0, [r1,#GPIOx_AFRH]  // Store result in GPIOB AFRH register


		ldr r1, =USART3_BASE       // Load USART3 BASE address to r1

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
	    tst r2, #(1 << 5)   // RXNE flag
	    beq RECV_LP
	    ldr r0, [r1, #USART_RDR]
  		pop {r1, r2, pc}


SEND_UART:
  		push {r1, r2, lr}
      	ldr r1, =USART3_BASE
SEND_LP:
      	ldr r2, [r1, #USART_ISR]
      	tst r2, #(1 << 7)   // TXE flag
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

