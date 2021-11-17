
/* Includes ------------------------------------------------------------------*/
#include "ad1955.h"

/* USER CODE BEGIN 0 */
uint16_t DAC_settings_L = Default_L;
uint16_t DAC_settings_R = Default_R;
uint8_t data;
uint16_t volumeCurrent = 100;
/* USER CODE END 0 */


/* USER CODE BEGIN 1 */


void DAC_Init(void)
{
	AD1955_ENABLE();
	MUTE_DISABLE();
	HAL_Delay(500);
	uint16_t dac_address_cmd = 0;

	if (ModeDac == StereoDualMono) {

	CSL_ON();
     dac_address_cmd = Default_L;
     HAL_SPI_Transmit(&hspi1, &dac_address_cmd, 1, 500);
 	CSL_OFF();



	CSR_ON();
     dac_address_cmd = Default_R;
     HAL_SPI_Transmit(&hspi1, &dac_address_cmd, 1, 500);
 	CSR_OFF();



	}




	if (ModeDac == MonoLeft) {
	CSL_ON();
     dac_address_cmd = Default_L;
     HAL_SPI_Transmit(&hspi1, &dac_address_cmd, 1, 500);
 	CSL_OFF();
	}


	if (ModeDac == MonoRight) {
	CSR_ON();
     dac_address_cmd = Default_R;
     HAL_SPI_Transmit(&hspi1, &dac_address_cmd, 1, 500);
 	CSR_OFF();
	}



}

void Set_Settings(uint8_t input,uint8_t sample_rate,uint8_t bit_rate,uint8_t mute,uint8_t volume)
{
	switch (input){
				case 0x00:
				break;
				case 0x01:
				break;
				case 0x02:
				break;
				}

				switch (sample_rate){
				case 0x00:
					if (bit_rate) {
					DAC_settings_L = Default_L;
					DAC_settings_R = Default_R;
					DAC_Write(DAC_settings_L, DAC_settings_R);
					} else {

						DAC_settings_L = Default_L|BR16;
						DAC_settings_R = Default_R|BR16;
						DAC_Write(DAC_settings_L, DAC_settings_R);
					}
				break;
				case 0x01:
					if (bit_rate) {
					DAC_settings_L = Default_L|SR96;
					DAC_settings_R = Default_R|SR96;
					DAC_Write(DAC_settings_L, DAC_settings_R);
				} else {

					DAC_settings_L = Default_L|SR96|BR16;
					DAC_settings_R = Default_R|SR96|BR16;
					DAC_Write(DAC_settings_L, DAC_settings_R);
				}
				break;
				case 0x02:
					if (bit_rate) {
					DAC_settings_L = SR192;
					DAC_settings_R = SR192;
					DAC_Write(DAC_settings_L, DAC_settings_R);
			} else {

				DAC_settings_L = DAC_settings_L|SR192|BR16;
				DAC_settings_R = DAC_settings_R|SR192|BR16;
				DAC_Write(DAC_settings_L, DAC_settings_R);
			}
				break;
				}

				switch (mute){
				case 0x00:
					MUTE_ENABLE();
				break;
				case 0x01:
					MUTE_DISABLE();
				break;
				}


//Volume
				volumeCurrent = volume * 0xA3;
				if(volume == 100) {
					DAC_Write(VolumeMax_L, VolumeMax_R);
				}
				if(volume == 0) {
					DAC_Write(Volume_L, Volume_R);
				}
				DAC_Write(volumeCurrent<<2|Volume_L, volumeCurrent<<2|Volume_R);

				}






void DAC_Write(uint16_t data_L, uint16_t data_R )
{

	uint16_t dac_address_cmd = 0;


	if (ModeDac == StereoDualMono) {



	CSL_ON();
     dac_address_cmd = data_L;
     HAL_SPI_Transmit(&hspi1, &dac_address_cmd, 1, 500);
 	CSL_OFF();



 	CSR_ON();
     dac_address_cmd = data_R;
     HAL_SPI_Transmit(&hspi1, &dac_address_cmd, 1, 500);
 	CSR_OFF();

	}

	if (ModeDac == MonoLeft) {
	CSL_ON();
     dac_address_cmd = data_L;
     HAL_SPI_Transmit(&hspi1, &dac_address_cmd, 1, 500);
 	CSL_OFF();
	}


	if (ModeDac == MonoRight) {
	CSR_ON();
     dac_address_cmd = data_R;
     HAL_SPI_Transmit(&hspi1, &dac_address_cmd, 1, 500);
 	CSR_OFF();
	}









}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
