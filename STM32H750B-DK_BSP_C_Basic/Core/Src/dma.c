/*
 * dma.c
 *
 *  Created on: Dec 8, 2022
 *      Author: patriciobulic
 */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define __FIFO
#define __BURST_4

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DMA_HandleTypeDef    			DMA2_UART1_Handle;
extern UART_HandleTypeDef 		UART1Handle;
MDMA_HandleTypeDef 				mdma_handle;
//extern SDRAM_HandleTypeDef 		sdramHand;
extern SDRAM_HandleTypeDef 		hsdram[1];
extern uint8_t 					mdma_complete;

/* Private function prototypes -----------------------------------------------*/
static void SDRAM_TransferComplete(DMA_HandleTypeDef *DmaHandle);
static void SDRAM_TransferError(DMA_HandleTypeDef *DmaHandle);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure the DMA controller according to the Stream parameters
  *         defined in dma.h file
  * @note  This function is used to :
  *        -1- Enable DMA2 clock
  *        -2- Select the DMA functional Parameters
  *        -3- Select the DMA instance to be used for the transfer
  *        -4- Initialize the DMA stream
  *        -5- Select Callbacks functions called after Transfer complete and
               Transfer error interrupt detection
  *        -6- Configure NVIC for DMA transfer complete/error interrupts
  *        -7- Start the DMA transfer using the interrupt mode
  * @param  None
  * @retval None
  */
HAL_StatusTypeDef DMA2_SDRAM_Config(DMA_HandleTypeDef* DmaHandle)
{
  /*## -1- Enable DMA2 clock #################################################*/
  __HAL_RCC_DMA2_CLK_ENABLE();

  /*##-2- Select the DMA functional Parameters ###############################*/
  DmaHandle->Init.Request = DMA_SDRAM_REQUEST;               /* MA_REQUEST_MEM2MEM               */
  DmaHandle->Init.Direction = DMA_MEMORY_TO_MEMORY;          /* M2M transfer mode                */
  DmaHandle->Init.PeriphInc = DMA_PINC_ENABLE;               /* Peripheral increment mode Enable */
  DmaHandle->Init.MemInc = DMA_MINC_ENABLE;                  /* Memory increment mode Enable     */
  DmaHandle->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* Peripheral data alignment : Word */
  DmaHandle->Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;    /* memory data alignment : Word     */
  DmaHandle->Init.Mode = DMA_NORMAL;                         /* Normal DMA mode                  */
  DmaHandle->Init.Priority = DMA_PRIORITY_HIGH;              /* priority level : high            */
#ifdef __FIFO
  DmaHandle->Init.FIFOMode = DMA_FIFOMODE_ENABLE;            /* FIFO mode enabled                */
  DmaHandle->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;   /* FIFO threshold: full             */
#else
  DmaHandle->Init.FIFOMode = DMA_FIFOMODE_DISABLE;           /* FIFO mode disabled                */
#endif
#ifdef __BURST_4
  DmaHandle->Init.MemBurst = DMA_MBURST_INC4;                /* Memory burst                     */
  DmaHandle->Init.PeriphBurst = DMA_PBURST_INC4;             /* Peripheral burst                 */
#else
  DmaHandle->Init.MemBurst = DMA_MBURST_SINGLE;              /* Memory burst                     */
  DmaHandle->Init.PeriphBurst = DMA_PBURST_SINGLE;           /* Peripheral burst                 */
#endif

  /*##-3- Select the DMA instance to be used for the transfer : DMA2_Stream1 #*/
  DmaHandle->Instance = DMA_SDRAM_INSTANCE;


  /*##-4- Initialize the DMA stream ##########################################*/
  if (HAL_DMA_Init(DmaHandle) != HAL_OK)
  {
    /* Initialization Error */
    return HAL_ERROR;
  }

  /*##-5- Select Callbacks functions called after Transfer complete and Transfer error */
  HAL_DMA_RegisterCallback(DmaHandle, HAL_DMA_XFER_CPLT_CB_ID, SDRAM_TransferComplete);
  HAL_DMA_RegisterCallback(DmaHandle, HAL_DMA_XFER_ERROR_CB_ID, SDRAM_TransferError);

  /*##-6- Configure NVIC for DMA transfer complete/error interrupts ##########*/
  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriority(DMA_SRAM_INSTANCE_IRQ, 0, 0);

  /* Enable the DMA STREAM global Interrupt */
  HAL_NVIC_EnableIRQ(DMA_SRAM_INSTANCE_IRQ);


  return HAL_OK;
}

/**
  * @brief  DMA conversion complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void SDRAM_TransferComplete(DMA_HandleTypeDef *DmaHandle)
{
  //TODO
}

/**
  * @brief  DMA conversion error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
static void SDRAM_TransferError(DMA_HandleTypeDef *DmaHandle)
{
  //TODO
}



HAL_StatusTypeDef MDMA_SDRAM_Config(void)
{

  /* Enable chosen MDMAx clock */
  SDRAM_MDMAx_CLK_ENABLE();

  /* Configure common MDMA parameters */
  mdma_handle.Init.Request = MDMA_REQUEST_SW;
  mdma_handle.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;			// Transfer a block
  mdma_handle.Init.Priority = MDMA_PRIORITY_HIGH;
  mdma_handle.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  mdma_handle.Init.SourceInc = MDMA_SRC_INC_WORD;
  mdma_handle.Init.DestinationInc = MDMA_DEST_INC_WORD;
  mdma_handle.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
  mdma_handle.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
  mdma_handle.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  mdma_handle.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  mdma_handle.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  mdma_handle.Init.BufferTransferLength = 128;							// Atomic transfer: 128 bytes
  mdma_handle.Init.SourceBlockAddressOffset = 0;
  mdma_handle.Init.DestBlockAddressOffset = 0;


  mdma_handle.Instance = SDRAM_MDMAx_CHANNEL;

   /* Associate the DMA handle */
  __HAL_LINKDMA(&hsdram[0], hmdma, mdma_handle);

  /* Deinitialize the stream for new transfer */
  HAL_MDMA_DeInit(&mdma_handle);

  /* Configure the DMA stream */
  if (HAL_MDMA_Init(&mdma_handle) != HAL_OK) {
	  return HAL_ERROR;
  }

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(SDRAM_MDMAx_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(SDRAM_MDMAx_IRQn);
  return HAL_OK;
}


void HAL_SDRAM_DMA_XferCpltCallback(MDMA_HandleTypeDef *hmdma){
	if (hmdma->Instance == SDRAM_MDMAx_CHANNEL){
		mdma_complete = 1;
	}
}



