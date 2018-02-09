/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "opamp.h"
#include "rng.h"
#include "rtc.h"
#include "sdmmc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ADXL345.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//
typedef enum {
	SD_READY = 0u, SD_SENDING_FIRSTHALF = 1u, SD_SENDING_SECONDHALF = 2u,
} SD_SEND_STATE;

//extern SD_HandleTypeDef hsd1;
HAL_SD_CardStatusTypedef status;
uint32_t wbytes; /* File write counts */
uint32_t byteswritten, bytesread;
uint16_t adcBuffer[numOfMicSamples] = { 0 }; /* File write buffer */
uint8_t *halfADCBuffer = (uint8_t *) adcBuffer + bufferSize / 2; //half of the buffer
uint8_t workBuffer[_MAX_SS];
uint8_t res;
volatile uint8_t sdSendState = SD_READY;
char randomNumberString[12] = { 0 };
char extension[] = "";
char filename[15] = { 0 };
char micFilename[10] = { 0 };
char accelFilename[10] = { 0 };
uint16_t micSDWriteCnt = 0u;
uint8_t micWriteSD = 0u;

//rtc
uint8_t aShowTime[8] = { 0 };

//uart
uint8_t uartData[50];
uint8_t errorCode = 0u;

//analog watchdog tresholds
uint16_t lt = 1500;
uint16_t ht = 2500;

//accelerometer
int16_t accelData[numOfAccelSamples][3] = { 0 };
uint16_t accelCurrentPos = 0u;
volatile uint8_t watermarkInt = 0u;
volatile uint16_t dataLeftInFifo = 0u;
uint8_t accelWriteSD = 0u;

//fatfs
FIL micSDFile;
FIL accelSDFile;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void startSampling();

void startSDCard();
HAL_StatusTypeDef startADC();
HAL_StatusTypeDef stopADC();
HAL_StatusTypeDef reConfigureAWD1(uint16_t lt, uint16_t ht);
void RTC_TimeShow(uint8_t* showtime);

void createFilename();
void createMicFilename();
void createAccelFilename();
HAL_StatusTypeDef openAndExpandMicFile();
HAL_StatusTypeDef openAndExpandAccelFile();

FRESULT writeHalfBuffer(const void* startAddress);
void micSDWritesFinished();
void accelSDWritesFinished();

FRESULT writeAccelData();
void readAccelData(uint8_t n);
void initAccel();
void startAccel();
void stopAccel();

//DAC
void startDAC();
void stopDAC();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_DMA_Init();
	MX_OPAMP2_Init();
	MX_ADC1_Init();
	MX_SDMMC1_SD_Init();
	MX_FATFS_Init();
	MX_TIM6_Init();
	MX_RTC_Init();
	MX_RNG_Init();
	MX_I2C3_Init();
	MX_USB_DEVICE_Init();
	MX_OPAMP1_Init();
	MX_DAC1_Init();
	/* USER CODE BEGIN 2 */
	//sprintf((char*) uartData, "reset\n");
	//HAL_UART_Transmit(&huart2, uartData, strlen(uartData), 100);
	//start SD card, mount file system
	startSDCard();
	//open and expand files for Mic and Accelerometer
	openAndExpandMicFile();
	openAndExpandAccelFile();
	//init accelerometer
	initAccel();
	//init channel biases
	startDAC();
	//start opamp(s)
	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	//start ADC in DMA circular mode
	startADC();
	//start timer
	HAL_TIM_Base_Start_IT(&htim6);
	//the system is ready to write SD card
	sdSendState = SD_READY; //place it!!!

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (micWriteSD == 1) {
			if (sdSendState == SD_SENDING_FIRSTHALF) {
				if (writeHalfBuffer(adcBuffer) == FR_OK) {
					if (sdSendState == SD_SENDING_FIRSTHALF) {
						sdSendState = SD_READY;
					}
					//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
					micSDWriteCnt++;
				}
			}
			if (sdSendState == SD_SENDING_SECONDHALF) {
				if (writeHalfBuffer(halfADCBuffer) == FR_OK) {
					if (sdSendState == SD_SENDING_SECONDHALF) {
						sdSendState = SD_READY;
					}
					//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
					micSDWriteCnt++;
				}
			}
			if (micSDWriteCnt >= numOfBufferWrites) {
				micSDWritesFinished();
			}
			if (sdSendState == SD_SENDING_FIRSTHALF
					|| sdSendState == SD_SENDING_SECONDHALF) {
				continue;
			}
		}
		if (accelWriteSD == 1) {
			//check accelerometer 
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET) {
				watermarkInt = 1u;
				dataLeftInFifo = 25u;
			} else {
				watermarkInt = 0u;
			}
			//if there are enough samples -> watermark 1 -> read one sample
			if (dataLeftInFifo > 0u) {
				readAccelData(1);
				dataLeftInFifo--;
			}
			//if we have enough data -> write to SD card and open new file
			if (accelCurrentPos >= numOfAccelSamples) {
				writeAccelData();
				accelSDWritesFinished();
			}
		}
	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Configure LSE Drive Capability
	 */
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
			|RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_I2C3
			|RCC_PERIPHCLK_USB|RCC_PERIPHCLK_SDMMC1
			|RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC;
	PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
	PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_MSI;
	PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_MSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the main internal regulator output voltage
	 */
	 if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	 {
		 _Error_Handler(__FILE__, __LINE__);
	 }

	 /**Configure the Systick interrupt time
	  */
	 HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	 /**Configure the Systick
	  */
	 HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	 /**Enable MSI Auto calibration
	  */
	 HAL_RCCEx_EnableMSIPLLMode();

	 /* SysTick_IRQn interrupt configuration */
	 HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	startSampling();
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc) {
	startSampling();
}

void startSampling() {
	if (micWriteSD == 0u) {
		//enable mic sd write flag
		micWriteSD = 1u;
		//disable analog watchdog interrupt
		NVIC_DisableIRQ(ADC1_2_IRQn);
		//debug
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
	}
	if (accelWriteSD == 0u) {
		//enable accel sd write flag
		accelWriteSD = 1u;
		//put it into measurement mode
		startAccel();
	}
}

void startSDCard() {
	//try to init SD card
	if ((errorCode = BSP_SD_Init()) != MSD_OK) {
		Error_Handler();
	}
#ifdef FATFS_MKFS_ALLOWED
	if((errorCode=f_mount(&SDFatFS, (TCHAR const*)SDPath, 0)) == FR_OK)
	{
		if ((errorCode=f_mkfs(SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer))) != FR_OK)
		{
			Error_Handler();
		}
	}
#else
	if ((errorCode = f_mount(&SDFatFS, (TCHAR const*) SDPath, 0)) != FR_OK) {
		Error_Handler();
	}
#endif
}

HAL_StatusTypeDef startADC() {
	if ((res = HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adcBuffer, numOfMicSamples))
			!= HAL_OK) {
		errorCode = (uint8_t) res;
		Error_Handler();
	}
	//enable AW interrupt
	NVIC_EnableIRQ(ADC1_2_IRQn);
	//__HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
	//__HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC);
	return res;
}

HAL_StatusTypeDef stopADC() {
	NVIC_DisableIRQ(ADC1_2_IRQn);
	HAL_StatusTypeDef res = HAL_ADC_Stop_DMA(&hadc1);
	return res;
}

HAL_StatusTypeDef reConfigureAWD1(uint16_t lt, uint16_t ht) {
	//stop ADC
	if (stopADC() != HAL_OK) {
		return HAL_ERROR;
	}
	//set new treshold values
	ADC_AnalogWDGConfTypeDef AnalogWDGConfig;
	AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
	AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
	AnalogWDGConfig.Channel = ADC_CHANNEL_15;
	AnalogWDGConfig.ITMode = ENABLE;
	AnalogWDGConfig.HighThreshold = ht;
	AnalogWDGConfig.LowThreshold = lt;
	//enable new configuraton
	if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK) {
		Error_Handler();
		return HAL_ERROR;
	}
	//restart ADC
	if (startADC() != HAL_OK) {
		return HAL_ERROR;
	}
	return HAL_OK;
}

void RTC_TimeShow(uint8_t* showtime) {
	RTC_DateTypeDef sdatestructureget;
	RTC_TimeTypeDef stimestructureget;

	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
	/* Display time Format : hh:mm:ss */
	sprintf((char*) showtime, "%02d%02d%02d%02d", sdatestructureget.Date,
			stimestructureget.Hours, stimestructureget.Minutes,
			stimestructureget.Seconds);
}

void createFilename() {
	RTC_TimeShow(aShowTime);
	//uint32_t randomNumber = 0;
	//HAL_RNG_GenerateRandomNumber(&hrng, &randomNumber);
	//sprintf(randomNumberString, "%lu", randomNumber);
	//sprintf(filename, "%.6s%.1s%s",aShowTime, randomNumberString, extension);
	//sprintf(filename, "%.8s%.1s", aShowTime, randomNumberString);
	sprintf(filename, "%.8s", aShowTime);
	//sprintf((char*) uartData, "%s\n", filename);
	//HAL_UART_Transmit(&huart2, uartData, strlen(uartData), 100);
}

void createMicFilename() {
	createFilename();
	sprintf(micFilename, "%.8sM", filename);
}

void createAccelFilename() {
	createFilename();
	sprintf(accelFilename, "%.8sA", filename);
}

HAL_StatusTypeDef openAndExpandMicFile() {
	createMicFilename();
	if ((errorCode = f_open(&micSDFile, micFilename,
			FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK) {
		//		if ((errorCode = f_expand(&micSDFile, micFileSize, 1)) != FR_OK) {
		//			Error_Handler();
		//			return HAL_ERROR;
		//		}
		if ((errorCode = f_lseek(&micSDFile, micFileSize)) != FR_OK) {
			Error_Handler();
			return HAL_ERROR;
		}
		if ((errorCode = f_lseek(&micSDFile, 0)) == FR_OK) {
			return HAL_OK;
		} else {
			Error_Handler();
			return HAL_ERROR;
		}
	} else {
		Error_Handler();
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef openAndExpandAccelFile() {
	createAccelFilename();
	if ((errorCode = f_open(&accelSDFile, accelFilename,
			FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK) {
		//try to use f_expand(file, size, operation mode(0/1))
		//		if ((errorCode = f_expand(&accelSDFile, accelFileSize, 1)) != FR_OK) {
		//			Error_Handler();
		//			return HAL_ERROR;
		//		}
		if ((errorCode = f_lseek(&accelSDFile, accelFileSize)) != FR_OK) {
			Error_Handler();
			return HAL_ERROR;
		}
		if ((errorCode = f_lseek(&accelSDFile, 0)) == FR_OK) {
			return HAL_OK;
		} else {
			Error_Handler();
			return HAL_ERROR;
		}
	} else {
		Error_Handler();
		return HAL_ERROR;
	}
}

FRESULT writeHalfBuffer(const void* startAddress) {
	//__HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
	//__HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC);
	if ((errorCode = f_write(&micSDFile, startAddress,
			(elementLength * numOfMicSamples) / 2, (void *) &wbytes)) == FR_OK) {
		return FR_OK;
	} else {
		Error_Handler();
		return FR_INT_ERR;
	}
}

void micSDWritesFinished() {
	//disable mic SD write flag
	micWriteSD = 0u;
	//close the reent file
	f_close(&micSDFile);
	//maybe modify treshold values
	if (reConfigureAWD1(lt, ht) != HAL_OK) {
		Error_Handler();
	}
	//debug
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
	//sprintf((char*) uartData, "fin\n", filename);
	//HAL_UART_Transmit(&huart2, uartData, strlen(uartData), 100);
	//open and expand a new file
	openAndExpandMicFile();
	//reset write counter
	micSDWriteCnt = 0u;
	//enable AW, we are ready
	NVIC_EnableIRQ(ADC1_2_IRQn);
}

void accelSDWritesFinished() {
	//close the related file
	f_close(&accelSDFile);
	//put accelerometer to standby mode
	stopAccel();
	//sprintf((char*) uartData, "accF\n", filename);
	//HAL_UART_Transmit(&huart2, uartData, strlen(uartData), 100);
	//prepare the next file
	openAndExpandAccelFile();
	//we have 0 sample now
	accelCurrentPos = 0u;
	//disable accel SD write flag
	accelWriteSD = 0u;
}

FRESULT writeAccelData() {
	if ((errorCode = f_write(&accelSDFile, accelData, accelFileSize,
			(void *) &wbytes)) == FR_OK) {
		return FR_OK;
	} else {
		Error_Handler();
		return FR_INT_ERR;
	}
}

void readAccelData(uint8_t n) {
	for (; n > 0; n--) {
		ADXL345_GetXyz(&accelData[accelCurrentPos][0],
				&accelData[accelCurrentPos][1], &accelData[accelCurrentPos][2]);
		accelCurrentPos++;
		if (accelCurrentPos >= numOfAccelSamples) {
			break;
		}
	}
}

//initialize and configure accelerometer
void initAccel() {
	uint8_t reg = 0u;
	HAL_I2C_MspInit(&hi2c3);
	ADXL345_Init(ADXL345_I2C_COMM);
	ADXL345_SetPowerMode(0);
	//Set data parameters such as data rate, measurement range, data format, and offset adjustment. 
	reg = ADXL345_GetRegisterValue(ADXL345_BW_RATE);
	reg = (reg & 0xF0) | 0x07; //0111 - 12,5Hz
	ADXL345_SetRegisterValue(ADXL345_BW_RATE, reg);
	//Configure interrupts (do not enable): thresholds and timing values, and map interrupts to pins. 
	ADXL345_SetRegisterValue(ADXL345_INT_MAP, 0x00); //all interrupts connected to INT1
	reg = ADXL345_GetRegisterValue(ADXL345_DATA_FORMAT);
	reg = (reg & 0xFC) | 0x03; //11 - +-16g
	ADXL345_SetRegisterValue(ADXL345_DATA_FORMAT, reg);

	//How to reset FIFO -> put to bypass mode maybe

	//Configure FIFO (if in use): mode, trigger interrupt if using trigger mode, and samples bits. 
	ADXL345_SetRegisterValue(ADXL345_FIFO_CTL, 0x00); //clears FIFO
	ADXL345_SetRegisterValue(ADXL345_FIFO_CTL, 0x59); //01- FIFO mode, 0 - trigger bit, 11001 - 25 samples (2 seconds) = 0101 1001 -> 0x59
	//Enable interrupts: INT_ENABLE register. 
	ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, 0x00);
	ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, ADXL345_WATERMARK); //enable Watermark
	//ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, ADXL345_DATA_READY); //enable data ready
}

//Place part into measurement mode: POWER_CTL register
void startAccel() {
	ADXL345_SetPowerMode(1);
}

//Place part into standby mode and clear FIFO
void stopAccel() {
	//disable interrupts
	ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, 0x00);
	//put to standby mode
	ADXL345_SetPowerMode(0);
	//reconfigure for fast wake up time
	ADXL345_SetRegisterValue(ADXL345_FIFO_CTL, 0x00); //clears FIFO, bypass mode
	ADXL345_SetRegisterValue(ADXL345_FIFO_CTL, 0x59); //01- FIFO mode, 0 - trigger bit, 11001 - 25 samples (2 seconds) = 0101 1001 -> 0x59
	dataLeftInFifo = 0u;
	ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, ADXL345_WATERMARK); //enable Watermark
}

void startDAC() {
	//Set DAC channel1 value: PIEZO x 16 channel bias
	if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4096 / 16)
			!= HAL_OK) {
		/* Setting value Error */
		Error_Handler();
	}
	//Enable DAC Channel1
	if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}
}

void stopDAC() {
	if (HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
		HAL_Delay(500);
		sprintf((char*) uartData, "E:%s%04d_%02d\n", file, line, errorCode);
		//HAL_UART_Transmit(&huart2, uartData, strlen((char*) uartData), 1000);
		NVIC_SystemReset();
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
