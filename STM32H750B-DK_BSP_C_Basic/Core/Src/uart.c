/*
 * uart.c
 *
 *  Created on: Dec 6, 2022
 *      Author: patriciobulic
 */

/* Includes ------------------------------------------------------------------*/
#include "uart.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

HAL_StatusTypeDef USART3_Init(UART_HandleTypeDef* huart3)
{

  huart3->Instance = USART3;
  huart3->Init.BaudRate = 115200;
  huart3->Init.WordLength = UART_WORDLENGTH_8B;
  huart3->Init.StopBits = UART_STOPBITS_1;
  huart3->Init.Parity = UART_PARITY_NONE;
  huart3->Init.Mode = UART_MODE_TX_RX;
  huart3->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  //huart1->Init.OverSampling = UART_OVERSAMPLING_16;
  //huart1->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  //huart1->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(huart3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

