
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AD1955_H__
#define __AD1955_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */



/* USER CODE BEGIN Private defines */
#define CSL_OFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
#define CSL_ON() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
#define CSR_OFF() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define CSR_ON() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
#define MUTE_ENABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define MUTE_DISABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define AD1955_ENABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define AD1955_DISABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)


#define DAC_Register_1 0x00
#define Power_Down 0x8000
#define Mute 0x4000
#define SR96 0x100
#define SR192 0x200
#define RJ 0x00
#define LJ 0x00
#define BR16 0xC
#define Default_L 0x800
#define Default_R 0xC00

#define DAC_Register_2 0x01
#define FS512 0x201
#define FS768 0x401

#define DAC_Register_L 0x02
#define Volume0_L 0x02
#define DAC_Register_R 0x03
#define Volume0_R 0x03


#define VolumeUp 0x333
#define VolumeMax_L 0x7FFF
#define VolumeMax_R 0xFFFF
#define Volume_L 0x02
#define Volume_R 0x03

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void DAC_Init(void);
void DAC_Write(uint16_t data_L, uint16_t data_R);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __AD1955_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
