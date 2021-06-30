/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC);
float _readADCVoltage_pinA(void);
float _readADCVoltage_pinB(void);
int _calibrate_phaseA(void);
int _calibrate_phaseB(void);
uint16_t _SPI_read(uint16_t _spi_data);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Enable_Pin8_Pin GPIO_PIN_3
#define Enable_Pin8_GPIO_Port GPIOF
#define Current_PhaseB_A2_Pin GPIO_PIN_3
#define Current_PhaseB_A2_GPIO_Port GPIOC
#define Current_PhaseB_A0_Pin GPIO_PIN_3
#define Current_PhaseB_A0_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define PWMC_Pin6_Pin GPIO_PIN_9
#define PWMC_Pin6_GPIO_Port GPIOE
#define PWMB_Pin5_Pin GPIO_PIN_11
#define PWMB_Pin5_GPIO_Port GPIOE
#define EncoderAU_Pin3_Pin GPIO_PIN_13
#define EncoderAU_Pin3_GPIO_Port GPIOE
#define EncoderAU_Pin3_EXTI_IRQn EXTI15_10_IRQn
#define PWMA_Pin9_Pin GPIO_PIN_14
#define PWMA_Pin9_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_14
#define SPI1_CS_GPIO_Port GPIOD
#define EncoderBV_Pin2_Pin GPIO_PIN_14
#define EncoderBV_Pin2_GPIO_Port GPIOG
#define EncoderBV_Pin2_EXTI_IRQn EXTI15_10_IRQn
#define EncoderIW_Pin11_Pin GPIO_PIN_5
#define EncoderIW_Pin11_GPIO_Port GPIOB
#define EncoderIW_Pin11_EXTI_IRQn EXTI9_5_IRQn
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define phaseA 1
#define phaseB 2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
