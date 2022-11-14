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

// Start of data section
 		.data

 		.align

STEV1: 	.word 	0x10   	// 32-bitna spr.
STEV2: 	.word 	0x40   	// 32-bitna spr.
VSOTA: 	.word 	0      	// 32-bitna spr.


// Start of text section
  		.text

  		.type  main, %function
  		.global main

   	   	.align
main:
  		ldr r0, =STEV1   // Naslov od STEV1 -> r0
  		ldr r1, [r0]    // Vsebina iz naslova v r0 -> r1

  		ldr r0, =STEV2   // Naslov od STEV1 -> r0
  		ldr r2, [r0]	// Vsebina iz naslova v r0 -> r2

  		add r3,r1,r2    // r1 + r2 -> r3

  		ldr r0, =VSOTA   // Naslov od STEV1 -> r0
  		str r3,[r0]		// iz registra r3 -> na naslov v r0

__end: 	b 	__end


