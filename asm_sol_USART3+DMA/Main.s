/*
 * Main.s
 *
 *  Created on: Jan 12, 2024
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

//-----------------------------
// DMA related definitions
//-----------------------------

//  AHB2ENR register offset is 0xDC (for enabling SRAMs)
.equ    RCC_AHB2ENR,  0xDC // RCC AHB2ENR peripheral clock reg

//  AHB1ENR register offset is 0xD8
.equ    RCC_AHB1ENR,  0xD8 // RCC AHB1ENR peripheral clock reg


// DMAMUX1 base address is 0x40020800
.equ    DMAMUX1_BASE,  0x40020800 // DMAMUX1 base address

.equ    DMAMUX1_C0CR,  0x00      // CR for Channel 0
.equ    DMAMUX1_C1CR,  0x04      // CR for Channel 1

// DMA1 base address is 0x40020000
.equ    DMA1_BASE,  0x40020000 // DMA1 base address
// DMA2 base address is 0x40020400
.equ    DMA2_BASE,  0x40020400 // DMA2 base address

.equ    DMA_USART3_TX_STREAM, 0    //Channel 0 on DMA1
.equ    DMA_USART3_RX_STREAM, 1    //Channel 1 on DMA1



// DMA Registers definitions
.equ    DMA_LISR, 0x00
.equ    DMA_HISR, 0x04
.equ    DMA_LIFCR, 0x08
.equ    DMA_HIFCR, 0x0C

.equ    DMA_SxCR_TX, 0x10 + 0x18 * DMA_USART3_TX_STREAM
.equ    DMA_SxFCR_TX, 0x24 + 0x18 * DMA_USART3_TX_STREAM
.equ DMA_SxSNDTR_TX, 0x14 + 0x18 * DMA_USART3_TX_STREAM
.equ    DMA_SxPAR_TX, 0x18 + 0x18 * DMA_USART3_TX_STREAM
.equ DMA_SxM0AR_TX, 0x1C + 0x18 * DMA_USART3_TX_STREAM


.equ    DMA_SxCR_RX, 0x10 + 0x18 * DMA_USART3_RX_STREAM
.equ    DMA_SxFCR_RX, 0x24 + 0x18 * DMA_USART3_RX_STREAM
.equ DMA_SxSNDTR_RX, 0x14 + 0x18 * DMA_USART3_RX_STREAM
.equ    DMA_SxPAR_RX, 0x18 + 0x18 * DMA_USART3_RX_STREAM
.equ DMA_SxM0AR_RX, 0x1C + 0x18 * DMA_USART3_RX_STREAM


// Comment following line when FLASH Linker Script is used
#define RAM_LinkScript




// Start of data section
 		.data

 		.align

LEDSTAT: .word 0 // LED state
MSECCNT: .word 0 //MSecs counter for SysTick_Handler
MSECMAX: .word 500 //MSecs interval for SysTick_Handler

#ifdef RAM_LinkScript
// Start of sram section
		.section .sram,"a",%progbits
#endif

		.align
NIZ1: .space 12

		.align
NIZ2: .asciz "Testni niz!"  // 12 bytes

.equ    NIZ_LEN, 12


// Start of text section
  		.text

  		.type  main, %function
  		.global main

   	   	.align
main:



#ifdef RAM_LinkScript

// Relocating Vector table to RAM (only for RAM Linker Script)
        bl RELLOC_VECTBL

/* Enable SRAMs and copy initial values only with RAM Linker script */
//  Enable SRAMs internal memories (for DMA1)
        bl  SRAM_ENABLE

//  Copy initial values for .sram section (from .text to SRAM)
		ldr r0,=_sisram
		ldr r1,=_ssram
		ldr r2,=_esram
		bl  MEM_COPY

#endif

        bl INIT_IO

		bl INIT_TC     // Priprava SysTick časovnika

   		bl INIT_USART3 // Priprava USART3 naprave

   		bl INIT_DMA    // Priprava DMA naprave za prenose preko USART3


// Main loop for USART3+DMA Echo test for blocks of 12 characters with change to upper case
loop:

    ldr r0,=NIZ2
    mov r1,#NIZ_LEN
    bl SND_DMA

	  bl LED_ON      // Vklop LED diode

	  mov r0,#500
//  bl DELAY // Zakasnitev SW Delay: r0 x 1msec
     bl DELAYTC // Zakasnitev SysTick : r0 x 1msec


    ldr r0,=NIZ1
    mov r1,#NIZ_LEN
    bl RCV_DMA

    ldr r0,=NIZ1
    ldr r1,=NIZ2
    ldr r2, =NIZ_LEN
    bl CHANGE


	bl LED_OFF      // Izlop LED diode

	mov r0,#500
//  bl DELAY // Zakasnitev SW Delay: r0 x 1msec
	bl DELAYTC // Zakasnitev SysTick : r0 x 1msec

	b loop          // skok na vrstico loop:



/*//   Main loop for echo test over USART3

loop:
       bl RECV_UART
       bl SEND_UART

       b loop          // skok na vrstico loop:*/

//    Main loop for sending character over USART every 0.5 sec
/*		mov r5,#'a'-1

loop:
	    add r5,r5,#1    // transmit chars from 'a' to 'z'
	    cmp r5,#'z'
	    bls cont
	    mov r5,#'a'

cont:  mov r0,r5
       bl SEND_UART


		mov r0,#500
		bl DELAY // Zakasnitev SW Delay: r0 x 1msec

		b loop          // skok na vrstico loop:*/


__end: 	b 	__end

RELLOC_VECTBL:

  	push {r0, r1, lr}

    ldr r1, =VTOR // Set Vector table addr. to 0x24000000
	ldr r0, =0x24000000
	str r0, [r1]

  	pop {r0, r1, pc}


SRAM_ENABLE:
  push {r5, r6, lr}

// Enable internal SRAMs (bits 31,30,29 in AHBeENR register)
	ldr r6, =RCC_BASE          // Load peripheral clock reg base address to r6
	ldr r5, [r6,#RCC_AHB2ENR]  // Read its content to r5
	orr r5, #(0b111 << 29)    // Set bits 31,30,29 to 1 to enable SRAMs clock
	str r5, [r6,#RCC_AHB2ENR]  // Store result in peripheral clock register

  pop  {r5, r6, pc}

MEM_COPY:
  push {r3, lr}

copy:
  ldr r3,[r0],#4
  str r3,[r1],#4

  cmp r1,r2
  blo copy

  pop  {r3, pc}


CHANGE:
    push {r3-r4,lr}

ch_zanka:
    ldrb r4, [r0], #1
    bic r3, r4, #0b100000  // zbrisi b5

    cmp r3, #'A'
    blo pisi

    cmp r3, #'Z'
    bhi pisi

    eor r4, r4, #0b100000  // spremeni crko

pisi:
    strb r4, [r1], #1  /* shranimo v niz2*/

    subs r2, r2, #1
    bne ch_zanka

    pop {r3-r4,pc}
    
    

INIT_DMA:
  push {r5, r6, lr}

    //-----------------------------------------
// ------------- DMA1 Settings

// Enable DMA1 Peripheral Clock (bit 0 in AHB1ENR register)
ldr r6, =RCC_BASE          // Load peripheral clock reg base address to r6
ldr r5, [r6,#RCC_AHB1ENR]  // Read its content to r5
orr r5, #1              // Set bit 0 to enable DMA1 clock
str r5, [r6,#RCC_AHB1ENR]  // Store result in peripheral clock register

ldr r6, =DMA1_BASE          // Load DMA1 BASE address to r6

// ------- DMA1 TX Settings
// Disable DMA TX Channel
ldr r5, [r6,#DMA_SxCR_TX]  // Read its content to r5
bic r5, #1
str r5, [r6,#DMA_SxCR_TX]    // Store result

wt0_EN0:
ldr r5, [r6,#DMA_SxCR_TX]  // wait until bit is read as 0
tst r5, #1
bne wt0_EN0


// Set MINC (increment memory pointer) and Direction (Memory->Peripheral)
orr r5,r5,#(0b10001 << 6)    // b10=1 and bit7,6 = 01
str r5, [r6,#DMA_SxCR_TX]    // Store result

// Enable direct mode
ldr r5, [r6,#DMA_SxFCR_TX]  // Read its content to r5
bic r5, #0b100
str r5, [r6,#DMA_SxFCR_TX]    // Store result

// ------- DMA1 RX Settings
  // Disable DMA RX Channel
ldr r5, [r6,#DMA_SxCR_RX]  // Read its content to r5
bic r5, #1
str r5, [r6,#DMA_SxCR_RX]    // Store result

wt1_EN0:
ldr r5, [r6,#DMA_SxCR_RX]  // wait until bit is read as 0
tst r5, #1
bne wt1_EN0

// Set MINC (increment memory pointer) and Direction (Memory<-Peripheral)
orr r5,r5,#(0b10000 << 6)    // b10=1 and bit7,6 = 00
str r5, [r6,#DMA_SxCR_RX]    // Store result

  // Enable direct mode
ldr r5, [r6,#DMA_SxFCR_RX]  // Read its content to r5
bic r5, #0b100
str r5, [r6,#DMA_SxFCR_RX]    // Store result

//----------------------------------------------
// ------------- DMAMUX1 Settings
// Set channels to devices translations (multiplexing)

ldr r6, =DMAMUX1_BASE        // Load reg base address to r6

mov r5,#46  // USART3_TX DMA Device Nr. is 46
    str r5, [r6, DMAMUX1_C0CR] // DMAREQ for Channel 0 to USART3_TX
mov r5,#45  // USART3_Rx DMA Device Nr. is 45
    str r5, [r6, DMAMUX1_C1CR] // DMAREQ for Channel 1 to USART3_RX

//----------------------------------------------
// ------------- USART3 Settings
ldr r6, =USART3_BASE      // Load USART3 BASE address to r1

// Disable USART3
ldr r5, [r6,#USART_CR1]  // Read its content to r5
bic r5, #1
str r5, [r6,#USART_CR1]  // Store result

// Enable DMA Transmit and Receive for USART3
    ldr r5, [r6, #USART_CR3]
orr r5, #(0b11<<6)          // Set bits 7 and 6 to enable DMAT and DMAR bits
str r5, [r6,#USART_CR3]    // Store result

// Enable USART3
ldr r6, =USART3_BASE      // Load USART3 BASE address to r6
ldr r5, [r6,#USART_CR1]  // Read its content to r5
orr r5, r5, #1
str r5, [r6,#USART_CR1]  // Store result


  pop {r5, r6, pc}


RCV_DMA:
  push {r5, r6, lr}

	ldr r6, =DMA1_BASE          // Load reg base address to r6

WAIT_EN: // Wait EN bit to become zero
      ldr r5, [r6, #DMA_SxCR_RX]
      tst r5, #1
      bne WAIT_EN

	ldr r6, =DMA1_BASE          // Load reg base address to r6

//  Receive (RX) DMA Init
	ldr r5, =USART3_BASE+USART_RDR // RX peripheral address to r5
	str r5, [r6,#DMA_SxPAR_RX]  // Store peripheral DMA pointer
	str r0, [r6,#DMA_SxM0AR_RX]  // Store address pointer
	str r1, [r6,#DMA_SxSNDTR_RX]  // Store number of units

// Clear flags in Status register
	mov r5,#(0b111101<<6)          // clear flags for Ch1 in ISR
	str r5, [r6,# DMA_LIFCR]      // Store

//  Enable DMA Channel
	ldr r5, [r6, #DMA_SxCR_RX]
	orr r5, r5, #1
    str r5, [r6, #DMA_SxCR_RX] // Enable channel

// Wait for the end of reception (TCIF
  ldr r6, =DMA1_BASE          // Load reg base address to r6

WAIT_RC:
    ldr r5, [r6, #DMA_LISR]
    tst r5, #(1 << 11)        // TCIF1 flag
    beq WAIT_RC

  pop {r5, r6, pc}


SND_DMA:
  	push {r5, r6, lr}

	ldr r6, =DMA1_BASE          // Load reg base address to r6

WAIT_EN1: // Wait EN bit to become zero
    ldr r5, [r6, #DMA_SxCR_TX]
    tst r5, #1
    bne WAIT_EN1

	ldr r6, =DMA1_BASE          // Load reg base address to r6
	//  Transmit (TX) DMA Init
	ldr r5, =USART3_BASE+USART_TDR // RX peripheral address to r5
	str r5, [r6,#DMA_SxPAR_TX]  // Store result in peripheral DMA pointerclock register
	str r0, [r6,#DMA_SxM0AR_TX]  // Store address pointer
	str r1, [r6,#DMA_SxSNDTR_TX]  // Store result in peripheral DMA pointerclock register

// Clear flags in Status register
	mov r5, #0b111101          // Clear all bits for channel 0
	str r5, [r6,#DMA_LIFCR]  // Store result in peripheral DMA pointerclock register

//  Enable DMA Channel
	ldr r6, =DMA1_BASE          // Load reg base address to r6
	ldr r5, [r6, #DMA_SxCR_TX]
	orr r5, r5, #1
    str r5, [r6, #DMA_SxCR_TX] // Enable channel

    // Wait for the end of transmission
    ldr r6, =USART3_BASE

WAIT_TC:
    ldr r5, [r6, #USART_ISR]
    tst r5, #(1 << 6)              // Test TC bit
    beq WAIT_TC

  pop {r5, r6, pc}



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

        .global DELAY

DELAY:
		push {r1,lr}

MSEC:   ldr r1,=MSDELAY
LOOP:   	subs r1,r1,#1
	        bne  LOOP

		subs r0,r0,#1
		bne  MSEC

		pop {r1,pc}

