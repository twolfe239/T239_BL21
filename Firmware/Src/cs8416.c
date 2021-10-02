
/* Includes ------------------------------------------------------------------*/
#include "cs8416.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


/* USER CODE BEGIN 1 */


void RCV_Init(void)
{
	CS8416_ENABLE();
uint8_t data=0;

data = R_RMCK256;
HAL_I2C_Mem_Write(&hi2c1, CS8416_ADDR, R_Control1, 1, &data, 1, 500);
data = R_I2S;
HAL_I2C_Mem_Write(&hi2c1, CS8416_ADDR, R_DataFormat, 1, &data, 1, 500);
data = R_SPDIF;
HAL_I2C_Mem_Write(&hi2c1, CS8416_ADDR, R_Control4, 1, &data, 1, 500);
	}

void DIR_XMOS(void)
{
	CS8416_DISABLE();
	uint8_t data=R_XMOS; //OR RST!
	HAL_I2C_Mem_Write(&hi2c1, CS8416_ADDR, R_Control4, 1, &data, 1, 500);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}
void DIR_CS8416(void)
{
	CS8416_ENABLE();
	uint8_t data = R_SPDIF;
	HAL_I2C_Mem_Write(&hi2c1, CS8416_ADDR, R_Control4, 1, &data, 1, 500);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
