/*
 * uart.h
 *
 *  Created on: Dec 6, 2022
 *      Author: patriciobulic
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32h7xx_hal.h"

HAL_StatusTypeDef USART3_Init(UART_HandleTypeDef* huart3);

#endif /* INC_UART_H_ */
