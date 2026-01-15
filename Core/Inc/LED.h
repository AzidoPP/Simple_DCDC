/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    LED.h
  * @brief   Header file for LED control functions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions prototypes ---------------------------------------------*/

/* LED1 control functions */
void LED1_ON(void);
void LED1_OFF(void);
void LED1_TURN(void);

/* LED2 control functions */
void LED2_ON(void);
void LED2_OFF(void);
void LED2_TURN(void);

/* LED3 control functions */
void LED3_ON(void);
void LED3_OFF(void);
void LED3_TURN(void);

/* LED4 control functions */
void LED4_ON(void);
void LED4_OFF(void);
void LED4_TURN(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
