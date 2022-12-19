/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stm32_lcd.h"
//#include "stm32h7xx_hal.h"
#include "stm32h750b_discovery.h"
#include "stm32h750b_discovery_lcd.h"
#include "stm32h750b_discovery_ts.h"
#include "stm32h750b_discovery_audio.h"
#include "stm32h750b_discovery_mmc.h"
#include "stm32h750b_discovery_sdram.h"
/* USER CODE END Includes */


/* USER CODE BEGIN ET */

/* Exported variables --------------------------------------------------------*/
extern const unsigned char stlogo[];
extern __IO uint32_t SRAMTest;
extern __IO uint32_t ButtonState ;

/* Exported types ------------------------------------------------------------*/
typedef enum {
  AUDIO_ERROR_NONE = 0,
  AUDIO_ERROR_NOTREADY,
  AUDIO_ERROR_IO,
  AUDIO_ERROR_EOF,
}AUDIO_ErrorTypeDef;

typedef struct
{
  void   (*DemoFunc)(void);
  uint8_t DemoName[50];
  uint32_t DemoIndex;
}BSP_DemoTypedef;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define SDRAM_WRITE_READ_ADDR        ((uint32_t)0xD0177000)

#define SDRAM_WRITE_READ_ADDR_OFFSET ((uint32_t)0x0800)

/* Defines for the Audio playing process */
#define PAUSE_STATUS     ((uint32_t)0x00) /* Audio Player in Pause Status */
#define RESUME_STATUS    ((uint32_t)0x01) /* Audio Player in Resume Status */
#define IDLE_STATUS      ((uint32_t)0x02) /* Audio Player in Idle Status */


#define LED_GREEN      LED1
#define LED_RED        LED2

#define AUDIO_REC_START_ADDR         SDRAM_WRITE_READ_ADDR
#define AUDIO_REC_TOTAL_SIZE         ((uint32_t) 0x0000E000)
#define AUDIO_RECPDM_START_ADDR      (AUDIO_REC_START_ADDR+AUDIO_REC_TOTAL_SIZE)

/* The Audio file is flashed with ST-Link Utility @ flash address =  AUDIO_SRC_FILE_ADDRESS */
#define AUDIO_SRC_FILE_ADDRESS       0x90080000   /* Audio file address in flash */
#define AUDIO_FILE_SIZE              0x80000

#define AUDIO_PLAY_SAMPLE        0
#define AUDIO_PLAY_RECORDED      1
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define COUNT_OF_EXAMPLE(x)    (sizeof(x)/sizeof(BSP_DemoTypedef))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
void Error_Handler(void);

void Touchscreen_demo1 (void);
void Touchscreen_demo2 (void);
void LCD_demo (void);
void Log_demo(void);
void Touchscreen_demo(void);
void AudioPlay_demo(void);
void Error_Handler(void);
void SDRAM_demo(void);
void SDRAM_DMA_demo (void);
void AudioPlay_demo (void);
void AudioRecord_demo(void);
uint8_t AUDIO_Process(void);
void QSPI_demo (void);
void MMC_demo (void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
uint8_t CheckForUserInput(void);
void Toggle_Leds(void);
void Touchscreen_DrawBackground_Circles(uint8_t state);
void Touchscreen_demo3(void);
uint8_t TouchScreen_GetTouchPosition(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FDCAN2_RX_Pin GPIO_PIN_5
#define FDCAN2_RX_GPIO_Port GPIOB
#define SDIO1_D2_Pin GPIO_PIN_10
#define SDIO1_D2_GPIO_Port GPIOC
#define MII_TX_EN_Pin GPIO_PIN_11
#define MII_TX_EN_GPIO_Port GPIOG
#define LCD_DISP_Pin GPIO_PIN_15
#define LCD_DISP_GPIO_Port GPIOJ
#define SDIO1_D3_Pin GPIO_PIN_11
#define SDIO1_D3_GPIO_Port GPIOC
#define FDCAN1_RX_Pin GPIO_PIN_14
#define FDCAN1_RX_GPIO_Port GPIOH
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define MII_TXD3_Pin GPIO_PIN_2
#define MII_TXD3_GPIO_Port GPIOE
#define MII_TXD1_Pin GPIO_PIN_12
#define MII_TXD1_GPIO_Port GPIOG
#define LCD_DISPD7_Pin GPIO_PIN_7
#define LCD_DISPD7_GPIO_Port GPIOD
#define SDIO1_CK_Pin GPIO_PIN_12
#define SDIO1_CK_GPIO_Port GPIOC
#define SDIO1_D5_Pin GPIO_PIN_9
#define SDIO1_D5_GPIO_Port GPIOB
#define SDIO1_D4_Pin GPIO_PIN_8
#define SDIO1_D4_GPIO_Port GPIOB
#define MII_TXD0_Pin GPIO_PIN_13
#define MII_TXD0_GPIO_Port GPIOG
#define SDIO1_CMD_Pin GPIO_PIN_2
#define SDIO1_CMD_GPIO_Port GPIOD
#define USB_OTG_FS2_ID_Pin GPIO_PIN_10
#define USB_OTG_FS2_ID_GPIO_Port GPIOA
#define VBUS_FS2_Pin GPIO_PIN_9
#define VBUS_FS2_GPIO_Port GPIOA
#define FDCAN1_TX_Pin GPIO_PIN_13
#define FDCAN1_TX_GPIO_Port GPIOH
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define SDIO1_D0_Pin GPIO_PIN_8
#define SDIO1_D0_GPIO_Port GPIOC
#define SDIO1_D1_Pin GPIO_PIN_9
#define SDIO1_D1_GPIO_Port GPIOC
#define USB_OTG_FS2_P_Pin GPIO_PIN_12
#define USB_OTG_FS2_P_GPIO_Port GPIOA
#define USB_OTG_FS2_N_Pin GPIO_PIN_11
#define USB_OTG_FS2_N_GPIO_Port GPIOA
#define MII_RX_ER_Pin GPIO_PIN_10
#define MII_RX_ER_GPIO_Port GPIOI
#define SDIO1_D7_Pin GPIO_PIN_7
#define SDIO1_D7_GPIO_Port GPIOC
#define SDIO1_D6_Pin GPIO_PIN_6
#define SDIO1_D6_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOI
#define LCD_INT_Pin GPIO_PIN_2
#define LCD_INT_GPIO_Port GPIOG
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define LCD_BL_Pin GPIO_PIN_0
#define LCD_BL_GPIO_Port GPIOK
#define MII_MDC_Pin GPIO_PIN_1
#define MII_MDC_GPIO_Port GPIOC
#define MII_TXD2_Pin GPIO_PIN_2
#define MII_TXD2_GPIO_Port GPIOC
#define MII_TX_CLK_Pin GPIO_PIN_3
#define MII_TX_CLK_GPIO_Port GPIOC
#define MII_CRS_Pin GPIO_PIN_2
#define MII_CRS_GPIO_Port GPIOH
#define MII_MDIO_Pin GPIO_PIN_2
#define MII_MDIO_GPIO_Port GPIOA
#define MII_RX_CLK_Pin GPIO_PIN_1
#define MII_RX_CLK_GPIO_Port GPIOA
#define MII_COL_Pin GPIO_PIN_3
#define MII_COL_GPIO_Port GPIOH
#define VCP_TX_Pin GPIO_PIN_10
#define VCP_TX_GPIO_Port GPIOB
#define VCP_RX_Pin GPIO_PIN_11
#define VCP_RX_GPIO_Port GPIOB
#define OTG_FS2_OverCurrent_Pin GPIO_PIN_11
#define OTG_FS2_OverCurrent_GPIO_Port GPIOH
#define MII_RX_DV_Pin GPIO_PIN_7
#define MII_RX_DV_GPIO_Port GPIOA
#define MII_RXD0_Pin GPIO_PIN_4
#define MII_RXD0_GPIO_Port GPIOC
#define MII_RX_D3_Pin GPIO_PIN_1
#define MII_RX_D3_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_2
#define LD1_GPIO_Port GPIOJ
#define LCD_RST_Pin GPIO_PIN_12
#define LCD_RST_GPIO_Port GPIOB
#define MII_RXD1_Pin GPIO_PIN_5
#define MII_RXD1_GPIO_Port GPIOC
#define MII_RX_D2_Pin GPIO_PIN_0
#define MII_RX_D2_GPIO_Port GPIOB
#define FDCAN2_TX_Pin GPIO_PIN_13
#define FDCAN2_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
