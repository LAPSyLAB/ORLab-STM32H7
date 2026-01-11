/*
 * assembler.s
 *
 */

  .syntax unified

  .text
  .global main_loop
  .extern draw_pixel
  .extern delay
  .extern SystemCoreClock
  .extern util_colors        // array barv definiran v touchscreen_asm.c

  color_index:
    .word 0                  // spremenljivka za trenutni index v arrayu

  colors_count:
    .word 25                  // spremenljivka stevilo barv v arrayu


main_loop:

	ldr r0, =SystemCoreClock
	ldr r0, [r0]

    // klic funkcije (z 3 parametri) draw pixel
	mov r0, #50
	mov r1, #50
	ldr r2, =0xFFFF0000
	bl draw_pixel

    // klic funkcije (z 1 parametrom) draw screen
	ldr r0, =0xFF000000
	bl draw_screen

    // klic funkcije (brez parametrov) fri_napis
	bl friNapis

	// prehod v zanko cakanja na dotik (da se ne izvaja cel program vsak cikel)
	b loop_wait_touch

  	b main_loop


loop_wait_touch:
    // klicanje funkicije, ki preveri stanje dotika
    bl touch_state
    cmp r0, #1  //ce funkcija vrne 1, je dotik zaznan
    beq touched

	ldr r0, =100000000 //po zakasnitvi ponovimo cikel
	bl delay
	b loop_wait_touch

touched:
    // ko je dotik zaznan, poklici funkcijo, ki iz seznama prebere naslednjo barvo
    bl get_next_color
    push {r0} 	// shrani rezultat funkcije (barvo) v r4

    // nariši kvadrat z izbrano barvo (poklici funkcijo z 5 parametri... peti parameter gre na sklad)
    mov r0, #360
    mov r1, #50
    mov r2, #40
    mov r3, #40
    bl draw_rect

    b loop_wait_touch



get_next_color:
    // nalozi trenutno vrendost indexa v r2
    ldr r1, =color_index
    ldr r2, [r1]

    // nalozi naslov tabele v r0
    ldr r0, =util_colors

    // poveca address za *4bajte (velikost enega elementa v tabeli)
    lsls r2, r2, #2
    adds r0, r0, r2

    ldr r0, [r0]

    // posodobi index za naslednjo barvo
    ldr r1, =color_index
    ldr r2, [r1]
    adds r2, r2, #1
    ldr r1, =colors_count
    ldr r1, [r1]
    cmp r2, r1
    bcc 1f
    movs r2, #0   // vrni index na 0, ko dosezemo stevilo barv

1:
    ldr r1, =color_index
    str r2, [r1]

	bx	lr

fri_napis:
  	mov r0, #70         // x position
	mov r1, #50         // y position
	mov r2, #120        // width
	mov r3, #40         // height
	ldr r4, =0xFFFFFFFF //color (white)
	push {r4}           // push 5th argoment onto stack
	bl draw_rect        // branch to funciton

	mov r0, #70
	mov r1, #110
	mov r2, #120
	mov r3, #40
	ldr r4, =0xFFFFFFFF
	push {r4}
	bl draw_rect

    mov r0, #70
    mov r1, #110
    mov r2, #40
    mov r3, #120
    ldr r4, =0xFFFFFFFF
    push {r4}
    bl draw_rect

    mov r0, #215
    mov r1, #110
    mov r2, #120
    mov r3, #40
    ldr r4, =0xFFFFFFFF
    push {r4}
    bl draw_rect

    mov r0, #215
    mov r1, #110
    mov r2, #40
    mov r3, #120
    ldr r4, =0xFFFFFFFF
    push {r4}
    bl draw_rect

    mov r0, #360
    mov r1, #110
    mov r2, #40
    mov r3, #120
    ldr r4, =0xFFFFFFFF
    push {r4}
    bl draw_rect

    mov r0, #360
    mov r1, #50
    mov r2, #40
    mov r3, #40
    ldr r4, =0xFFFFFFFF
    push {r4}
    bl draw_rect

    bx   lr

// delay(r0 = count, rough cycles = r0 * loop_cost)
// zakasnitvena funkcija
delay:
    subs r0, r0, #1   // decrement
    bne  delay        // loop until zero
    bx    lr
