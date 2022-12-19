/* Includes ------------------------------------------------------------------*/
#include "app_timers.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TIM3_CCR3 200U-1U
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * Init and start TIM3 in Interrupt mode
 *
 * */

HAL_StatusTypeDef Init_TIM3(TIM_HandleTypeDef* TIMHandle) {

	RCC_ClkInitTypeDef    clkconfig;
	volatile uint32_t uwTimclock = 0U;
	volatile uint32_t uwPrescalerValue = 0U;
	volatile uint32_t uwAPB1Prescaler = 0U;
	volatile uint32_t SysClockFreq;
	volatile uint32_t HCLKFreq;
	volatile uint32_t APB1Freq;
	volatile uint32_t APB2Freq;

	/*Configure the TIM3 IRQ priority */
	HAL_NVIC_SetPriority(TIM3_IRQn, TICK_INT_PRIORITY, 0U);
	/* Enable the TIM3 global Interrupt */
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	/* Enable TIM6 clock */
	__HAL_RCC_TIM3_CLK_ENABLE();

	// Find out the TIM3 CLK frequency:
    SysClockFreq = HAL_RCC_GetSysClockFreq();
	HCLKFreq = HAL_RCC_GetHCLKFreq();
	APB1Freq = HAL_RCC_GetPCLK1Freq();
	//APB2Freq = HAL_RCC_GetPCLK2Freq();
	/* Get APB1 prescaler */
	uwAPB1Prescaler = clkconfig.APB1CLKDivider;
	/* Compute TIMERs clock */
	if (uwAPB1Prescaler == RCC_HCLK_DIV1)
	{
		uwTimclock = APB1Freq; // if PPRE1 bits in RCC_CFGR are 0XX (APB1 Prescaler = 1), then TIMs run at APB1 clk)
	}
	else
	{
	  uwTimclock = 2*APB1Freq; // if PPRE1 bits in RCC_CFGR are 1XX (APB1 Prescaler > 1), then TIMs run at 2*APB1 clk)
	}

	/* Compute the prescaler value to have TIM3 counter clock equal to 1 MHz */
	uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);
	/* Initialize TIM3 */
	TIMHandle->Instance = TIM3;
	TIMHandle->Init.Period = 65535U;
	TIMHandle->Init.Prescaler = uwPrescalerValue;
	TIMHandle->Init.ClockDivision = 0;
	TIMHandle->Init.CounterMode = TIM_COUNTERMODE_UP;
	TIMHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if(HAL_TIM_Base_Init(TIMHandle) == HAL_OK)
	{
		/* Start the TIM time Base generation in interrupt mode */
		//return HAL_TIM_Base_Start_IT(TIMHandle);
		return HAL_TIM_Base_Start(TIMHandle);
	}
	/* Return function status */
	return HAL_ERROR;
}


