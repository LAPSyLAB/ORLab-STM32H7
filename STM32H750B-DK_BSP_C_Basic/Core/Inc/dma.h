/*
 * dma.h
 *
 *  Created on: Dec 8, 2022
 *      Author: patriciobulic
 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"


/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* DMA stream parameters definitions for M2M transfers between SRAM and SDRAM.
 *   You can modify these parameters to select */
/*   a different DMA Stream and/or channel.                                   */
/* But note that only DMA2 Streams are capable of Memory to Memory transfers. */
#define DMA_SDRAM_INSTANCE               DMA2_Stream3
#define DMA_SDRAM_REQUEST                DMA_REQUEST_MEM2MEM
#define DMA_SRAM_INSTANCE_IRQ            DMA2_Stream3_IRQn
#define DMA_SDRAM_INSTANCE_IRQHANDLER    DMA2_Stream3_IRQHandler

/* MDMA definitions for SDRAM ;DMA transfer */
#define SDRAM_MDMAx_CLK_ENABLE             __HAL_RCC_MDMA_CLK_ENABLE
#define SDRAM_MDMAx_CLK_DISABLE            __HAL_RCC_MDMA_CLK_DISABLE
#define SDRAM_MDMAx_CHANNEL                MDMA_Channel0
#define SDRAM_MDMAx_IRQn                   MDMA_IRQn
#define SDRAM_MDMA_IRQHandler              MDMA_IRQHandler

/* USER CODE END EC */

extern SDRAM_HandleTypeDef 			sdramHand;


HAL_StatusTypeDef DMA2_SDRAM_Config(DMA_HandleTypeDef* DmaHandle);
HAL_StatusTypeDef MDMA_SDRAM_Config(void);


#endif /* INC_DMA_H_ */
