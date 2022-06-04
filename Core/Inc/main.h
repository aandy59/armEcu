/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "delay10us.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define ignCoilPinState (GPIOB->ODR & GPIO_ODR_OD13)
#define ignCoilOn LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_13) //катушка
#define ignCoilOff LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_13)

#define leftInjectorPinState (GPIOB->ODR & GPIO_ODR_OD14)
#define leftInjectorOn LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_14) //лева€ форсунка
#define leftInjectorOff LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_14)

#define rightInjectorPinState (GPIOB->ODR & GPIO_ODR_OD15)
#define rightInjectorOn LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_15) //права€ форсунка
#define rightInjectorOff LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_15)

#define mainReleyPinState (GPIOB->ODR & GPIO_ODR_OD19)
#define mainReleyOn LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_9) //главное реле
#define mainReleyOff LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_9)

#define fuelPumpReleyPinState (GPIOB->ODR & GPIO_ODR_OD8)
#define fuelPumpReleyOn LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_8) //реле топливного насоса
#define fuelPumpReleyOff LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_8)

#define lambdaHeaterPinState (GPIOA->ODR & GPIO_ODR_OD15)
#define lambdaHeaterOn LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_15) //нагреватель ƒ 
#define lambdaHeaterOff LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_15)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
