/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t input = 0;  //target 1
uint8_t rrmck = 0;
uint8_t sample_rate = 0; //target 2
uint8_t drmck = 0;
uint8_t bit_rate = 0;  //target 4
uint8_t volume = 20; //target 5
uint16_t volumeCurrent = 20;
uint8_t mute = 0; //target 3
uint8_t target = 1;
uint8_t ENTstatus = 0;
uint8_t flPin = intPinDef;
uint16_t DAC_settings_L = Default_L;
uint16_t DAC_settings_R = Default_R;
uint8_t data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

//	flIntrpt = 1;

	if (GPIO_Pin == GPIO_PIN_0) {
		flPin = intPinENT;
	}

	if (GPIO_Pin == GPIO_PIN_2) {
		flPin = intPinR;
	}

	if (GPIO_Pin == GPIO_PIN_3) {
		flPin = intPinL;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	//------------------------------------------------------------------ LCD
	lcd_Init();
	lcd_Clear();

//	sprintf(buf0, "P: %.2f hPa", var);
	lcd_Goto(0, 0);
	lcd_PrintC("AGENDA");
	//------------------------------------------------------------------ Audio DEInit
	CS8416_DISABLE();
	AD1955_DISABLE();
	I2S_DISABLE();
	CSL_OFF();
	CSR_OFF();
	MUTE_ENABLE();
	//------------------------------------------------------------------ Audio Init
	DAC_Init();
	RCV_Init();
	I2S_ENABLE();
	DIR_CS8416();
	//------------------------------------------------------------------ Knock-knock-knock
	SPON();
	HAL_Delay(50);
	SPOFF();
	HAL_Delay(50);
	SPON();
	HAL_Delay(45);
	SPOFF();

	lcd_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		switch (flPin) {
		case intPinENT:
			flPin = intPinDef;
			if(ENTstatus == mainM) {
				ENTstatus = selectM;


				lcd_Goto(3, 10);
				lcd_PrintC("MENU");
				HAL_Delay(100);
				lcd_Clear();
			}
			if(ENTstatus == applyM) {
				lcd_Goto(3, 10);
				lcd_PrintC("APPLY");
				HAL_Delay(100);
				lcd_Clear();
				ENTstatus = mainM;
				//WRITE SETTINGS!
				switch (input){
				case 0x00:
					DIR_XMOS();
				break;
				case 0x01:
					DIR_CS8416();
					 data = R_SPDIF;
					HAL_I2C_Mem_Write(&hi2c1, CS8416_ADDR, R_Control4, 1, &data, 1, 500);
				break;
				case 0x02:
					DIR_CS8416();
					 data = R_TOSLINK;
					HAL_I2C_Mem_Write(&hi2c1, CS8416_ADDR, R_Control4, 1, &data, 1, 500);
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
				volumeCurrent = volume * 0x333;
				if(volume == 20) {
					DAC_Write(VolumeMax_L, VolumeMax_R);
				}
				if(volume == 0) {
					DAC_Write(Volume_L, Volume_R);
				}
				DAC_Write(volumeCurrent<<2|Volume_L, volumeCurrent<<2|Volume_R);
			}


			break;





		case intPinL:  // LEFT BUTTON
			flPin = intPinDef;
			if (ENTstatus){
// IN SETTINGS
				switch (target) {
				case inputM:
					ENTstatus = applyM;
					input == 0 ? input = 2 : input--;
					break;
				case sample_rateM:
					ENTstatus = applyM;
					sample_rate == 0 ? sample_rate = 2 : sample_rate--;
					break;
				case muteM:
					ENTstatus = applyM;
					mute == 0 ? mute = 1 : mute--;
					break;
				case bit_rateM:
					ENTstatus = applyM;
					bit_rate == 0 ? bit_rate = 1 : bit_rate--;
					break;
				case volumeM:
					ENTstatus = applyM;
					if (volume == 0) volume = 0;
					else volume--;
					break;
					default:
					break;
				}


			} else {
				//cycle MENU
				target == 1 ? target = 5 : target--;
				if (target==0) target = 1;
				ENTstatus = mainM;
			}
			break;


		case intPinR:  // RIGHT BUTTON


			flPin = intPinDef;
			if (ENTstatus){
// IN SETTINGS
				switch (target) {
				case inputM:
					ENTstatus = applyM;
					input == 2 ? input = 0 : input++;
					break;
				case sample_rateM:
					ENTstatus = applyM;
					sample_rate == 2 ? sample_rate = 0 : sample_rate++;
					break;
				case muteM:
					ENTstatus = applyM;
					mute == 1 ? mute = 0 : mute++;
					break;
				case bit_rateM:
					ENTstatus = applyM;
					bit_rate == 1 ? bit_rate = 0 : bit_rate++;
					break;
				case volumeM:
					ENTstatus = applyM;
					if (volume == 20) volume = 20;
					else volume++;
					break;
					default:
					break;
				}

			} else {
				//cycle MENU
			target == 5 ? target = 1 : target++;
			ENTstatus = mainM;
			break;
			}


		case intPinDef:  // LCD Overview



switch (target){

case 0x01:
	lcd_Goto(0, 8);
	lcd_PrintC("<");
break;
case 0x02:
	lcd_Goto(0, 19);
	lcd_PrintC("<");
break;
case 0x03:
	lcd_Goto(1, 8);
	lcd_PrintC("<");
break;
case 0x04:
	lcd_Goto(1, 19);
	lcd_PrintC("<");
break;





}
switch (input){
case 0x00:
	lcd_Goto(0, 0);
	lcd_PrintC("USB");
break;
case 0x01:
	lcd_Goto(0, 0);
	lcd_PrintC("S/PDIF");
break;
case 0x02:
	lcd_Goto(0, 0);
	lcd_PrintC("TOSLINK");
break;
}

switch (sample_rate){
case 0x00:
	lcd_Goto(0, 10);
	lcd_PrintC("48kHz ");
break;
case 0x01:
	lcd_Goto(0, 10);
	lcd_PrintC("96kHz ");
break;
case 0x02:
	lcd_Goto(0, 10);
	lcd_PrintC("192kHz");
break;
}

switch (mute){
case 0x00:
	lcd_Goto(1, 0);
	lcd_PrintC("MUTE");
break;
case 0x01:
	lcd_Goto(1, 0);
	lcd_PrintC("PLAY");
break;
}

switch (bit_rate){
case 0x00:
	lcd_Goto(1, 10);
	lcd_PrintC("16 BIT");
break;
case 0x01:
	lcd_Goto(1, 10);
	lcd_PrintC("24 BIT");
break;
}

for (uint8_t i = 0; i < volume; i++ )
{
	lcd_Goto(2, i);
	lcd_PrintC(0xFF);
	//lcd_Data(0xFF);

}


Time();
HAL_IWDG_Refresh(&hiwdg);
			break;





		default:
			flPin = intPinDef;
			break;
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
