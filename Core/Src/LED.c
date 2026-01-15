/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    LED.c
  * @brief   Source file for LED control functions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "LED.h"

/* Private function prototypes -----------------------------------------------*/
/* None needed */

/* Exported functions --------------------------------------------------------*/

/* LED1 control functions ----------------------------------------------------*/

/**
  * @brief  Turn on LED1
  * @retval None
  */
void LED1_ON(void)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Turn off LED1
  * @retval None
  */
void LED1_OFF(void)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Toggle LED1 state
  * @retval None
  */
void LED1_TURN(void)
{
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

/* LED2 control functions ----------------------------------------------------*/

/**
  * @brief  Turn on LED2
  * @retval None
  */
void LED2_ON(void)
{
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Turn off LED2
  * @retval None
  */
void LED2_OFF(void)
{
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Toggle LED2 state
  * @retval None
  */
void LED2_TURN(void)
{
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}

/* LED3 control functions ----------------------------------------------------*/

/**
  * @brief  Turn on LED3
  * @retval None
  */
void LED3_ON(void)
{
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Turn off LED3
  * @retval None
  */
void LED3_OFF(void)
{
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Toggle LED3 state
  * @retval None
  */
void LED3_TURN(void)
{
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
}

/* LED4 control functions ----------------------------------------------------*/

/**
  * @brief  Turn on LED4
  * @retval None
  */
void LED4_ON(void)
{
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Turn off LED4
  * @retval None
  */
void LED4_OFF(void)
{
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Toggle LED4 state
  * @retval None
  */
void LED4_TURN(void)
{
    HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
}

/* USER CODE BEGIN 1 */
/* User code can be added here if needed */
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
