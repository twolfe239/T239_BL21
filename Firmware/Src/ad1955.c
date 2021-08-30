
/* Includes ------------------------------------------------------------------*/
#include "ad1955.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


/* USER CODE BEGIN 1 */


void DAC_Init(void)
{
	AD1955_ENABLE();
	MUTE_DISABLE();
	HAL_Delay(500);
	uint16_t dac_address_cmd = 0;
	CSL_ON();
     dac_address_cmd = Default_L;
     HAL_SPI_Transmit(&hspi1, &dac_address_cmd, 1, 500);
 	CSL_OFF();
	CSR_ON();
     dac_address_cmd = Default_R;
     HAL_SPI_Transmit(&hspi1, &dac_address_cmd, 1, 500);
 	CSR_OFF();
}


void DAC_Write(uint16_t data_L, uint16_t data_R )
{
	uint16_t dac_address_cmd = 0;
	CSL_ON();
     dac_address_cmd = data_L;
     HAL_SPI_Transmit(&hspi1, &dac_address_cmd, 1, 500);
 	CSL_OFF();
	CSR_ON();
     dac_address_cmd = data_R;
     HAL_SPI_Transmit(&hspi1, &dac_address_cmd, 1, 500);
 	CSR_OFF();
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
