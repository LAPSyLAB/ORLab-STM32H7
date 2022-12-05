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

// If the processor configuration includes the DWT profiling counters, the instruction count can be calculated as:

// instructions executed = DWT_CYCCNT - DWT_CPICNT - DWT_EXCCNT - DWT_SLEEPCNT - DWT_LSUCNT + DWT_FOLDCNT

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

    	mov r0,#0
    	ldr r1, =DWT_BASE


// Disable DWT Counters
		ldr r2, [r1,#DWT_CTRL]
		bic r2,r2,#1      // Disabling CYCCNTENA bit
		str r2, [r1,#DWT_CTRL]

// Reset DWT Counters
		str r0, [r1,#DWT_CYCCNT]
		str r0, [r1,#DWT_CPICNT]
		str r0, [r1,#DWT_EXCCNT]
		str r0, [r1,#DWT_SLPCNT]
		str r0, [r1,#DWT_LSUCNT]
		str r0, [r1,#DWT_FOLDCNT]

		ldr r0, [r1,#DWT_CYCCNT]


// Prepare initial registers for the code measurement
		ldr r5,=64000

// Enable DWT Counters
		ldr r2, [r1,#DWT_CTRL]
		orr r2,r2,#1      // Enabling CYCCNTENA bit
		str r2, [r1,#DWT_CTRL]

// Read DWT Counter before value
		ldr r0, [r1,#DWT_CYCCNT]

// To try: ISB  ; Instruction Synchronisation Barrier

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
		bic r2,r2,#1      // Disabling CYCCNTENA bit
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

		// r8 contains number of instructions

		b main

__end: 	b 	__end


