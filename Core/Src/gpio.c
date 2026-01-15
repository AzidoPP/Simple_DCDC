/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "cmsis_os.h"
#include "shared_data.h"
#include "LED.h"

extern osMessageQueueId_t buttonQueueHandle;
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NMOS1_Pin|NMOS2_Pin|NMOS3_Pin|PP2_EN_Pin
                          |PP3_EN_Pin|LED1_Pin|LED2_Pin|LED3_Pin
                          |LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PP2_IN_Pin|PP3_IN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BUTTON_UP_Pin */
  GPIO_InitStruct.Pin = BUTTON_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NMOS1_Pin NMOS2_Pin NMOS3_Pin PP2_EN_Pin
                           PP3_EN_Pin LED1_Pin LED2_Pin LED3_Pin
                           LED4_Pin */
  GPIO_InitStruct.Pin = NMOS1_Pin|NMOS2_Pin|NMOS3_Pin|PP2_EN_Pin
                          |PP3_EN_Pin|LED1_Pin|LED2_Pin|LED3_Pin
                          |LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PP2_IN_Pin PP3_IN_Pin */
  GPIO_InitStruct.Pin = PP2_IN_Pin|PP3_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_MID_Pin BUTTON_LEFT_Pin BUTTON_DOWN_Pin */
  GPIO_InitStruct.Pin = BUTTON_MID_Pin|BUTTON_LEFT_Pin|BUTTON_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_RIGHT_Pin */
  GPIO_InitStruct.Pin = BUTTON_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_RIGHT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BtnEvent_t evt;
    BaseType_t awoken = pdFALSE;
	LED1_TURN();
    evt.tick = xTaskGetTickCountFromISR();

    if (GPIO_Pin == BUTTON_UP_Pin) {
        evt.key_id = 0;
        evt.type = (HAL_GPIO_ReadPin(BUTTON_UP_GPIO_Port, BUTTON_UP_Pin) == GPIO_PIN_RESET)
                   ? BTN_EVENT_PRESS : BTN_EVENT_RELEASE;
    }
    else if (GPIO_Pin == BUTTON_DOWN_Pin) {
        evt.key_id = 1;
        evt.type = (HAL_GPIO_ReadPin(BUTTON_DOWN_GPIO_Port, BUTTON_DOWN_Pin) == GPIO_PIN_RESET)
                   ? BTN_EVENT_PRESS : BTN_EVENT_RELEASE;
    }
    else if (GPIO_Pin == BUTTON_LEFT_Pin) {
        evt.key_id = 2;
        evt.type = (HAL_GPIO_ReadPin(BUTTON_LEFT_GPIO_Port, BUTTON_LEFT_Pin) == GPIO_PIN_RESET)
                   ? BTN_EVENT_PRESS : BTN_EVENT_RELEASE;
    }
    else if (GPIO_Pin == BUTTON_RIGHT_Pin) {
        evt.key_id = 3;
        evt.type = (HAL_GPIO_ReadPin(BUTTON_RIGHT_GPIO_Port, BUTTON_RIGHT_Pin) == GPIO_PIN_RESET)
                   ? BTN_EVENT_PRESS : BTN_EVENT_RELEASE;
    }
    else if (GPIO_Pin == BUTTON_MID_Pin) {
        evt.key_id = 4;
        evt.type = (HAL_GPIO_ReadPin(BUTTON_MID_GPIO_Port, BUTTON_MID_Pin) == GPIO_PIN_RESET)
                   ? BTN_EVENT_PRESS : BTN_EVENT_RELEASE;
    } else {
        return;
    }

    // put event into queue
    osMessageQueuePut(buttonQueueHandle, &evt, 0, 0); /* 非阻塞 */
    portYIELD_FROM_ISR(awoken);
}
/* USER CODE END 2 */
