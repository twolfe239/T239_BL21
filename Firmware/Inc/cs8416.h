
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CS8416_H__
#define __CS8416_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */



/* USER CODE BEGIN Private defines */
#define CS8416_ADDR 0x20
#define CS8416_ENABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define CS8416_DISABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define R_Control1 0x01
#define R_RMCK256 0x00
#define R_RMCK128 0x02

#define R_Control4 0x04
#define R_TOSLINK 0x80
#define R_SPDIF 0x88
#define R_XMOS 0x40

#define R_DataFormat 0x05
#define R_LJ 0x00
#define R_I2S 0x05
#define R_RJ 0x88




/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void RCV_Init(void);
void DIR_CS8416(void);
void DIR_XMOS(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CS8416_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
