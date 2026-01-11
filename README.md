# ORLab-STM32H7
Organizacija računalnikov (VSP): Vsebine za delo s STM32H7 ploščo.
Kratke razlage vsebine map.

## Docs	

Osnovna dokumentacija za delo s sistemom STM32H750-DK.

## stm32h7-asm

Osnovni projekt CubeIDE in VSCode v zbirniku za stm32h7.
Osnova za delo na LAB vajah.

## asm_sol_xxxx

Rešitve posameznih LAB vaj (vsebujejo samo main.s, ne celotnega projekta)
Main.s se preprosto zamenja v stm32h7-asm projektu.

## STM32H750B-DK_C_Basic

Osnovni projekt CubeIDE v C za stm32h7 (generiran s CubeMX).

## STM32H750B-DK_BSP_C_Basic

Osnovni projekt CubeIDE v C za stm32h7 (na osnovi BSP paketa - ročno sestavljen, ni generiran s CubeMX).
Na osnovi projekta H7-BSP-LCD-OS avtorja P. Bulića. 
Vsebuje tudi FreeRTOS in nekatere audio funkcije.

## STM32H750B-DK_BSP_Touch_Demo
Osnovni projekt CubeIDE v C za stm32h7 (na osnovi BSP paketa - ročno sestavljen, ni generiran s CubeMX).
Na osnovi projekta H7-BSP-LCD-OS avtorja P. Bulića. 
Vsebuje osnovno demonstracijo uporabe ekrana in zaznave dotikov.

Nahaja se v okviru projektov za predmet VIN na naslovu: https://github.com/LAPSyLAB/STM32H7_Discovery_VIN_Projects/tree/main/STM32H750-DK_BSP_Touch_Demo

## STM32H750B-DK_BSP_ASM_Touch_Demo

Izpeljava projekta STM32H750-DK_BSP_Touch_Demo z glavno zanko v zbirniku (main_loop v main_asm.s) in 
demonstracijo klicev prisotnih funkcij v C iz zbirnika.
Narejen je izpis osnovnega ekrana in zaznava dotika (se spremeni barva pike na črki i).

