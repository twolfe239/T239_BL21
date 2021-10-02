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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "ssd1306.h"
#include "i2c_lcd.h"
#include "ad1955.h"
#include "cs8416.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "stm32f1xx_hal_flash.h"
#include "crc.h"

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
#define ENTER_Pin GPIO_PIN_0
#define ENTER_GPIO_Port GPIOA
#define ENTER_EXTI_IRQn EXTI0_IRQn
#define R_Pin GPIO_PIN_2
#define R_GPIO_Port GPIOA
#define R_EXTI_IRQn EXTI2_IRQn
#define L_Pin GPIO_PIN_3
#define L_GPIO_Port GPIOA
#define L_EXTI_IRQn EXTI3_IRQn
#define SPI1_CSL_Pin GPIO_PIN_6
#define SPI1_CSL_GPIO_Port GPIOA
#define ENI2S_Pin GPIO_PIN_2
#define ENI2S_GPIO_Port GPIOB
#define DIRI2S_Pin GPIO_PIN_10
#define DIRI2S_GPIO_Port GPIOB
#define RSTD_Pin GPIO_PIN_11
#define RSTD_GPIO_Port GPIOB
#define MUTE_Pin GPIO_PIN_12
#define MUTE_GPIO_Port GPIOB
#define RSTR_Pin GPIO_PIN_13
#define RSTR_GPIO_Port GPIOB
#define SPI1_CSR_Pin GPIO_PIN_15
#define SPI1_CSR_GPIO_Port GPIOB
#define BUZZ_Pin GPIO_PIN_8
#define BUZZ_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_4
#define ENC_A_GPIO_Port GPIOB
#define ENC_B_Pin GPIO_PIN_5
#define ENC_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */





#define I2S_DISABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)
#define I2S_ENABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)
#define RCV_DISABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define RCV_ENABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define SPON() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define SPOFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define intPinENT 0x00
#define intPinR 0x02
#define intPinL 0x03
#define intPinDef 0x1F


//MENU
#define mainM 0x00
#define selectM 0x01
#define applyM 0x02

#define inputM 0x01
#define sample_rateM 0x02
#define muteM 0x03
#define bit_rateM 0x04
#define volumeM 0x05

#define sel "<"
#define SettingsPage 0x0801FC00
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
