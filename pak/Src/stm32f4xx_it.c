/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"

//char str_tx[2];
char str_tx[21];
uint8_t pt = 169;
//extern unsigned char faza = 201;
//extern unsigned char fazatemp = 201; //faza defoult=5ms
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
//void EXTI1_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI1_IRQn 0 */
//////////  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
////////  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
////////  HAL_TIM_Base_Stop(&htim2);
////////  faza = fazatemp;
////////  HAL_TIM_Base_Start(&htim2);
////////
//////// // __HAL_TIM_SET_AUTORELOAD(&htim2, 3000);
//////// 
//  /* USER CODE END EXTI1_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
//  /* USER CODE BEGIN EXTI1_IRQn 1 */
////////
//  /* USER CODE END EXTI1_IRQn 1 */
//}

///**
//  * @brief This function handles TIM2 global interrupt.
//  */
//void TIM2_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM2_IRQn 0 */
////////  HAL_TIM_Base_Start(&htim3);
////////  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//  /* USER CODE END TIM2_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim2);
//  /* USER CODE BEGIN TIM2_IRQn 1 */
////////
//  /* USER CODE END TIM2_IRQn 1 */
//}

///**
//  * @brief This function handles TIM3 global interrupt.
//  */
//void TIM3_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM3_IRQn 0 */
////////  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
////////  HAL_TIM_Base_Start(&htim4);
//  /* USER CODE END TIM3_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim3);
//  /* USER CODE BEGIN TIM3_IRQn 1 */
////////
//  /* USER CODE END TIM3_IRQn 1 */
//}

///**
//  * @brief This function handles TIM4 global interrupt.
//  */
//void TIM4_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM4_IRQn 0 */
////////   HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
//  /* USER CODE END TIM4_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim4);
//  /* USER CODE BEGIN TIM4_IRQn 1 */
////////
//  /* USER CODE END TIM4_IRQn 1 */
//}

///**
//  * @brief This function handles TIM5 global interrupt.
//  */
//void TIM5_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM5_IRQn 0 */
////////	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//  /* USER CODE END TIM5_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim5);
//  /* USER CODE BEGIN TIM5_IRQn 1 */
////////
//  /* USER CODE END TIM5_IRQn 1 */
//}

///**
//  * @brief This function handles USB On The Go FS global interrupt.
//  */
//void OTG_FS_IRQHandler(void)
//{
//  /* USER CODE BEGIN OTG_FS_IRQn 0 */
////
//  /* USER CODE END OTG_FS_IRQn 0 */
//  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
//  /* USER CODE BEGIN OTG_FS_IRQn 1 */
////
//  /* USER CODE END OTG_FS_IRQn 1 */
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
