/*
 * Main.s
 *
 *  Created on: Aug 24, 2022
 *      Author: rozman
 *
 * Update 12/2024
 *		+all counters work
 *		+number of instructions calculated
 */

		  .syntax unified
		  .cpu cortex-m7
		  .thumb

///////////////////////////////////////////////////////////////////////////////
// Meritve
///////////////////////////////////////////////////////////////////////////////

//-------------- subs,bne --------------------------
// 		ldr r5,=N
// ------------- odsek kode ------------------------
// tloop:  subs r5,r5,#1
//         bne  tloop
// ------------- konec kode ------------------------
/*
// Timings - usually in second or more repetition (on first one cycles are higher)
╔═══════╤════════════════╗
║ N     │ DWT_CYCCNT(1st)║
╠═══════╪════════════════╣
║ 50    │ 56     (78)    ║
╟───────┼────────────────╢
║ 100   │ 106    (128)   ║
╟───────┼────────────────╢
║ 200   │ 206            ║
╟───────┼────────────────╢
║ 500   │ 506            ║
╟───────┼────────────────╢
║ 1000  │ 1006           ║
╟───────┼────────────────╢
║ 64000 │ 64006 (64028)  ║
╚═══════╧════════════════╝
Comment: difference : r5 instructions are 16bit, r8 instructions are 32 bit, but both with same timing.
		 if nop is added, for N=64000, results are 96030 and 96006
Conclusion: branch prediction is the main influencer here.
*/


///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////
// Definitions section. Define all the registers and
// constants here for code readability.

// Constants

// Register Addresses

	.equ     DWT_BASE,   	0xE0001000 // DWT Base address

	.equ     DWT_CTRL,   	0x00 // DWT_CTRL   reg (RM0433, pp.3209)
	.equ     DWT_CYCCNT,   	0x04 // increments on each clock cycle when the processor is not halted in debug state.
	.equ     DWT_CPICNT,   	0x08 // additional cycles required to execute multi-cycle instructions, and instruction fetch stalls
	.equ     DWT_EXCCNT,   	0x0C // count the total cycles spent in interrupt processing (cycles spent performing exception entry and exit procedures)
	.equ     DWT_SLPCNT,   	0x10 // count the total number of cycles during which the processor is sleeping (cycles spent sleeping)
	.equ     DWT_LSUCNT,   	0x14 // counts the total number of cycles that the processor is processing an LSU operation (cycles spent waiting for loads and stores to complete)
								 // For example, an LDR that takes two cycles to complete increments this counter one cycle.
								 // Equivalently, an LDR that stalls for two cycles (and so takes four cycles), increments counter three times.
	.equ     DWT_FOLDCNT,   0x18 // count the total number of folded instructions (cycles saved by instructions which execute in zero cycles)
								 // This counts 1 for each instruction that takes 0 cycles.

	.equ     DWT_CTRL_ENABLE_CNTs, 0x003f0001  // Enable bits 16-21 and 1

//
// If the processor configuration includes the DWT profiling counters, the instruction count can be calculated as:
// instructions executed = DWT_CYCCNT - DWT_CPICNT - DWT_EXCCNT - DWT_SLEEPCNT - DWT_LSUCNT + DWT_FOLDCNT
	.equ     DWT_LAR,   	0xFB0 	// DWT_LAR  DWT_LAR = 0xC5ACCE55; // unlock (CM7)
	.equ     DEMCR,   	    0xE000EDFC // SCB_DEMCR |= 0x01000000;

// Start of data section
 		.data

 		.align

STEV1: 	.word 	0x10   	// 32-bitna spr.
STEV2: 	.word 	0x20   	// 32-bitna spr.
STEV3: 	.word 	0x30   	// 32-bitna spr.
STEV4: 	.word 	0x40   	// 32-bitna spr.


// Start of text section
  		.text

  		.type  main, %function
  		.global main

   	   	.align
main:

    	ldr r1, =DWT_BASE

		//bl  INIT_CNT
		//bl RESET_CNT

// Prepare initial registers for the code measurement
		ldr r5,=50

// Enable DWT Counters
		ldr r2, [r1,#DWT_CTRL]
		ldr r0,=DWT_CTRL_ENABLE_CNTs // Enabling all Counters bits
		orr r2,r2,r0
		str r2, [r1,#DWT_CTRL]

// Read DWT Counter before value
		ldr r0, [r1,#DWT_CYCCNT]

// ------------------------------
// Start of measurement code - use registers r3+ only !
// ------------------------------
tloop:  subs r5,r5,#1
        bne  tloop

// ------------------------------
//   End of measurement code
// ------------------------------

// Read DWT Counter after value
		ldr r2, [r1,#DWT_CYCCNT]

		sub r0,r2,r0   // Difference in r0

// Disable DWT Counters
		ldr r2, [r1,#DWT_CTRL]
		ldr r3,=DWT_CTRL_ENABLE_CNTs // Enabling all Counters bits
		bic r2,r2,r3     // Disabling all counter bits
		str r2, [r1,#DWT_CTRL]

// Read other DWT Counters

		ldr r2, [r1,#DWT_CPICNT]
		sub r8,r0,r2

		ldr r3, [r1,#DWT_EXCCNT]
		sub r8,r8,r3

		ldr r4, [r1,#DWT_SLPCNT]
		sub r8,r8,r4

		ldr r5, [r1,#DWT_LSUCNT]
		sub r8,r8,r5

		ldr r6, [r1,#DWT_FOLDCNT]
		add r8,r8,r6

		// r0 contains number of cycles
		// r8 contains calculated number of instructions

		b main

__end: 	b 	__end

INIT_CNT:
	  	push {r0-r2, lr}


		// Added in 2024 (but it seems not needed for H7):
//		ldr r1,=DWT_BASE
//		ldr r0,=0xC5ACCE55
//		str r0,[r1,#DWT_LAR]   // *DWT_LAR = 0xC5ACCE55; // unlock (CM7)

//		bit [24]	TRCENA Global enable for all DWT and ITM features:
//				0 = DWT and ITM blocks disabled.
//				1 = DWT and ITM blocks enabled.
		ldr r1,=DEMCR
		ldr r0,[r1]
		orr r0,r0,#0x01000000
		str r0,[r1]            // *SCB_DEMCR |= 0x01000000;
		// End: Added in 2024 :


    	ldr r1, =DWT_BASE

// Disable DWT Counters
		ldr r2, [r1,#DWT_CTRL]
		// bic r2,r2,#1      // Disabling CYCCNTENA bit
		ldr r0,=DWT_CTRL_ENABLE_CNTs // Mask for enabling all Counters bits
		bic r2,r2,r0     // Disabling all counter bits
		str r2, [r1,#DWT_CTRL]

// Reset DWT Counters
    	mov r0,#0
		str r0, [r1,#DWT_CYCCNT]
		str r0, [r1,#DWT_CPICNT]
		str r0, [r1,#DWT_EXCCNT]
		str r0, [r1,#DWT_SLPCNT]
		str r0, [r1,#DWT_LSUCNT]
		str r0, [r1,#DWT_FOLDCNT]

	  	pop {r0-r2, pc}

RESET_CNT:
	  	push {r0-r2, lr}

	  	ldr r1, =DWT_BASE


// Disable DWT Counters
		ldr r2, [r1,#DWT_CTRL]
		// bic r2,r2,#1      // Disabling CYCCNTENA bit
		ldr r0,=DWT_CTRL_ENABLE_CNTs // Enabling all Counters bits
		bic r2,r2,r0     // Disabling all counter bits
		str r2, [r1,#DWT_CTRL]

// Reset DWT Counters
	  	mov r0,#0
		str r0, [r1,#DWT_CYCCNT]
		str r0, [r1,#DWT_CPICNT]
		str r0, [r1,#DWT_EXCCNT]
		str r0, [r1,#DWT_SLPCNT]
		str r0, [r1,#DWT_LSUCNT]
		str r0, [r1,#DWT_FOLDCNT]

        pop {r0-r2, pc}

CALC_CNT: // DWT Counter is in r0
	  	push {r1-r6,r8, lr}

    	ldr r1, =DWT_BASE

// Disable DWT Counters
		ldr r2, [r1,#DWT_CTRL]
		// bic r2,r2,#1      // Disabling CYCCNTENA bit
		ldr r3,=DWT_CTRL_ENABLE_CNTs // Enabling all Counters bits
		bic r2,r2,r3     // Disabling all counter bits
		str r2, [r1,#DWT_CTRL]

// instructions executed = DWT_CYCCNT - DWT_CPICNT - DWT_EXCCNT - DWT_SLEEPCNT - DWT_LSUCNT + DWT_FOLDCNT
// Read other DWT Counters

		ldr r2, [r1,#DWT_CPICNT]
		sub r8,r0,r2

		ldr r3, [r1,#DWT_EXCCNT]
		sub r8,r8,r3

		ldr r4, [r1,#DWT_SLPCNT]
		sub r8,r8,r4

		ldr r5, [r1,#DWT_LSUCNT]
		sub r8,r8,r5

		ldr r6, [r1,#DWT_FOLDCNT]
		add r8,r8,r6

		// r8 contains number of instructions
		mov r0,r8

	  	pop {r1-r6,r8, pc}


ENABLE_CNT:
	  	push {r0-r2, lr}

	  	ldr r1, =DWT_BASE

// Enable DWT Counters
		ldr r2, [r1,#DWT_CTRL]
//		orr r2,r2,#1      // Enabling CYCCNTENA bit
		ldr r0,=DWT_CTRL_ENABLE_CNTs // Enabling all Counters bits
		orr r2,r2,r0
		str r2, [r1,#DWT_CTRL]

	  	pop {r0-r2, pc}



