/**
  ******************************************************************************
  * @file    stm32g4xx_mc_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control for the STM32G4 Family.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup STM32G4xx_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_config.h"
#include "mc_tasks.h"
#include "ui_task.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32g4xx_hal.h"

#include "stm32g4xx.h"

/* USER CODE BEGIN Includes */
#include "schedule.h"
/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32G4xx_IRQ_Handlers STM32G4xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY / 1000)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void ADC1_2_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);

void SPD_TIM_M1_IRQHandler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  * @retval None
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */

  // Clear Flags M1
  LL_ADC_ClearFlag_JEOS( ADC1 );

  // Highfrequency task
  TSK_HighFrequencyTask();
 /* USER CODE BEGIN HighFreq */

 /* USER CODE END HighFreq  */

 /* USER CODE BEGIN ADC1_2_IRQn 1 */

 /* USER CODE END ADC1_2_IRQn 1 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_UP_M1_IRQHandler(void)
{
 /* USER CODE BEGIN TIMx_UP_M1_IRQn 0 */

 /* USER CODE END  TIMx_UP_M1_IRQn 0 */

    LL_TIM_ClearFlag_UPDATE(TIM1);
    R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);

 /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

 /* USER CODE END  TIMx_UP_M1_IRQn 1 */
}

void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */
  if (LL_TIM_IsActiveFlag_BRK(TIM1))
  {
    LL_TIM_ClearFlag_BRK(TIM1);
    R3_2_BRK_IRQHandler(&PWM_Handle_M1);
  }
  if (LL_TIM_IsActiveFlag_BRK2(TIM1))
  {
    LL_TIM_ClearFlag_BRK2(TIM1);
    R3_2_BRK2_IRQHandler(&PWM_Handle_M1);
  }
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_Scheduler();

  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */
}

/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  * @retval None
  */
void SPD_TIM_M1_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */

 /* Encoder Timer UPDATE IT is dynamicaly enabled/disabled, checking enable state is required */
  if (LL_TIM_IsEnabledIT_UPDATE (ENCODER_M1.TIMx) && LL_TIM_IsActiveFlag_UPDATE (ENCODER_M1.TIMx))
  {
    LL_TIM_ClearFlag_UPDATE(ENCODER_M1.TIMx);
    ENC_IRQHandler(&ENCODER_M1);
    /* USER CODE BEGIN M1 ENCODER_Update */

    /* USER CODE END M1 ENCODER_Update   */
  }
  else
  {
  /* No other IT to manage for encoder config */
  }
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
 /* USER CODE BEGIN HardFault_IRQn 0 */

 /* USER CODE END HardFault_IRQn 0 */
  TSK_HardwareFaultTask();

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {

  }
 /* USER CODE BEGIN HardFault_IRQn 1 */

 /* USER CODE END HardFault_IRQn 1 */

}

void SysTick_Handler(void)
{

#ifdef MC_HAL_IS_USED
static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  if (SystickDividerCounter == SYSTICK_DIVIDER)
  {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    SystickDividerCounter = 0;
  }
  SystickDividerCounter ++;
#endif /* MC_HAL_IS_USED */

  /* USER CODE BEGIN SysTick_IRQn 1 */
  HQ_SYSTICK_IRQHandler();
  /* USER CODE END SysTick_IRQn 1 */
    MC_RunMotorControlTasks();

  /* USER CODE BEGIN SysTick_IRQn 2 */
  /* USER CODE END SysTick_IRQn 2 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
