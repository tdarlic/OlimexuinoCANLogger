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
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "fatfs.h"
#include <string.h>
#include "file_utils.h"
#ifdef USE_USB
#include "usb_device.h"
#endif

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;
I2C_HandleTypeDef hi2c2;
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

osThreadId defaultTaskHandle;

unsigned char bWriteFault = 0; // in case of overlap or write fault

typedef struct CAN_BitrateSetting_t {
	uint32_t bitrate;
	uint32_t Prescaler; /*!< Specifies the length of a time quantum.
	 This parameter must be a number between Min_Data = 1 and Max_Data = 1024 */

	uint32_t BS1; /*!< Specifies the number of time quanta in Bit Segment 1.
	 This parameter can be a value of @ref CAN_time_quantum_in_bit_segment_1 */

	uint32_t BS2; /*!< Specifies the number of time quanta in Bit Segment 2.
	 This parameter can be a value of @ref CAN_time_quantum_in_bit_segment_2 */

} CAN_BitrateSetting;
// @formatter:off
/**
 * Calculations taken from webpage:
 * http://www.bittiming.can-wiki.info/
 * With following settings
 * Type: bxCAN, Clock: 72MHz, max brp: 1024,
 * SP: 87.5%, min tq: 8, max tq: 25, FD factor: undefined, SJW: 1
 */
const CAN_BitrateSetting CAN_BitrateSettingsArray[8] = {
		//brp, pre, BS1, BS2
		{ 500, 9, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 250, 18, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 125, 36, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 100, 45, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 83, 54, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 50, 90, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 20, 225, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 10, 450, CAN_BS1_13TQ, CAN_BS2_2TQ }
};
// @formatter:on

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define STRLINE_LENGTH 					1024
#define SD_WRITE_BUFFER             	(1024*5)// 5k
#define SD_WRITE_BUFFER_FLUSH_LIMIT 	(1024*4)// 4k
#define MMCSD_BLOCK_SIZE 				512

char sLine[STRLINE_LENGTH];

FATFS g_sFatFs;
FIL* logfile;

uint32_t stLastWriting;

unsigned char bLogging = 0; // if =1 than we logging to SD card

int iFilterMask = 0;
int iFilterValue = 0;
unsigned char bLogStdMsgs = 1;
unsigned char bLogExtMsgs = 1;
unsigned char bIncludeTimestamp = 1;

// buffer for collecting data to write
char sd_buffer[SD_WRITE_BUFFER];
WORD sd_buffer_length = 0;

// buffer for storing ready to write data
char sd_buffer_for_write[SD_WRITE_BUFFER];
unsigned char bReqWrite = 0; // write request, the sd_buffer is being copied to sd_buffer_for_write
WORD sd_buffer_length_for_write = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C2_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_RTC_Init(void);
static void MX_CRC_Init(void);
void StartDefaultTask(void const * argument);
void blinkThread(void const *argument);
static FRESULT mountSDCard(void);
static int read_config_file(void);
static int align_buffer(void);
static void copy_buffer(void);
static void request_write(void);
static void writeToSDBuffer(char *pString);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	osThreadId blinkTID;
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
	MX_USART2_UART_Init();
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	MX_CAN_Init();
	MX_I2C2_Init();
	MX_USB_PCD_Init();
	MX_RTC_Init();
	MX_CRC_Init();

	/* USER CODE BEGIN 2 */
	/* init code for FATFS */
	MX_FATFS_Init();
	mountSDCard();
	read_config_file();
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */
	osThreadDef(blink, blinkThread, osPriorityNormal, 0, 100);
	blinkTID = osThreadCreate(osThread(blink), NULL);
	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/* USER CODE BEGIN 4 */

void blinkThread(void const *argument) {
	while (1) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		osDelay(500);
	}
	osThreadTerminate(NULL);
}

static FRESULT mountSDCard(void) {
	return f_mount(&g_sFatFs, "0:", 0);
}

// fill buffer with spaces (before \r\n) to make it 512 byte size
// return 1 if filled and ready to write
/**
 * @brief Fill buffer with spaces (before \r\n) to make it 512 byte size
 * @return 1 if filled and ready to write
 */
static int align_buffer(void)
{
	int i;
	int len;

	if (sd_buffer_length < 2)
		return 0;
	if (sd_buffer[sd_buffer_length - 2] != '\r')
		return 0;
	if (sd_buffer[sd_buffer_length - 1] != '\n')
		return 0;

	len = MMCSD_BLOCK_SIZE - (sd_buffer_length % MMCSD_BLOCK_SIZE);
	for (i = 0; i < len; i++)
		sd_buffer[sd_buffer_length + i - 2] = ' ';
	sd_buffer[sd_buffer_length - 2] = ',';
	sd_buffer[sd_buffer_length + len - 2] = '\r';
	sd_buffer[sd_buffer_length + len - 1] = '\n';

	sd_buffer_length += len;

	return 1;
}

/**
 * @brief copy input buffer into the buffer for flash writing data
 */
static void copy_buffer(void)
{
	// request write operation
	memcpy(sd_buffer_for_write, sd_buffer, sd_buffer_length);
	sd_buffer_length_for_write = sd_buffer_length;
	sd_buffer_length = 0;
}

/**
 * @brief Requests write
 */
static void request_write(void)
{
	if (bReqWrite)
		bWriteFault = 1; // buffer overlapping

	// request write operation
	align_buffer();
	copy_buffer();
	bReqWrite = 1;
}

/**
 * Writes string to SD write buffer
 * @param pString
 */
static void writeToSDBuffer(char *pString)
{
	WORD length = strlen(pString);

	// Add string
	memcpy(&sd_buffer[sd_buffer_length], pString, length);
	sd_buffer_length += length;

	// Check flush limit
	if (sd_buffer_length >= SD_WRITE_BUFFER_FLUSH_LIMIT)
	{
		request_write();
	}
}

/**
 * @brief Test function used successfully to write to Sd card
 */
void WriteToSD(void) {
	char buffer[128];

	static FRESULT fresult;
	FIL file;
	int len;
	UINT bytes_written;

	/* init code for FATFS */
	//MX_FATFS_Init();
	//mount SD card
	fresult = f_mount(&g_sFatFs, "0:", 0);

	//open file on SD card
	fresult = f_open(&file, "0:file.txt", FA_OPEN_ALWAYS | FA_WRITE);

	if (fresult == FR_OK) {
		//go to the end of the file
		fresult = f_lseek(&file, file.fsize);
	}

	//generate some string
	len = sprintf(buffer, "Hello World!\r\n");

	//write data to the file
	fresult = f_write(&file, buffer, 15, &bytes_written);

	//close file
	fresult = f_close(&file);
}

/**
 * @brief Reads the configuration file and then loads the settings into the
 * @brief CAN peripheral and restart it
 * @return 0 on fault
 */
static int read_config_file(void) {
	FRESULT fresult;
	FIL file;
	int value;
	char name[128];
	int baud;
	int res = 0;
	int ack = 0;

	iFilterMask = 0;
	iFilterValue = 0;
	bIncludeTimestamp = 1;
	bLogStdMsgs = 1;
	bLogExtMsgs = 1;

	uint8_t brs;

	//fresult = f_mount(&g_sFatFs, "0:", 0);

	//open file on SD card
	fresult = f_open(&file, "0:Config.txt", FA_OPEN_EXISTING | FA_READ);
	// if result is not FR_OK then there was an error opening
	if (fresult != FR_OK)
		return 0;

	while (f_gets(sLine, STRLINE_LENGTH, &file)) {
		if (sscanf(sLine, "%s %d", name, &value) == 0) {
			continue;
		}

		if (strcmp(name, "baud") == 0) {
			baud = value;
			res = 1; // at least we got baudrate, config file accepted
		} else if (strcmp(name, "ack_en") == 0) {
			ack = value;
		} else if (strcmp(name, "id_filter_mask") == 0) {
			iFilterMask = value;
		} else if (strcmp(name, "id_filter_value") == 0) {
			iFilterValue = value;
		} else if (strcmp(name, "timestamp") == 0) {
			bIncludeTimestamp = value;
		} else if (strcmp(name, "log_std") == 0) {
			bLogStdMsgs = value;
		} else if (strcmp(name, "log_ext") == 0) {
			bLogExtMsgs = value;
		}
	}

	// configure CAN
	brs = get_CAN_setBaudeRate(baud);

	if (HAL_CAN_DeInit(&hcan) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	hcan.Init.Prescaler = CAN_BitrateSettingsArray[brs].Prescaler;
	hcan.Init.BS1 = CAN_BitrateSettingsArray[brs].BS1;
	hcan.Init.BS2 = CAN_BitrateSettingsArray[brs].BS2;
	if (ack) {
		hcan.Init.Mode = CAN_MODE_NORMAL;
	} else {
		hcan.Init.Mode = CAN_MODE_SILENT;
	}
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	f_close(&file);

	return res;
}

/**
 * @brief Prepare file on SD card for writing to
 */
void start_log()
{
	RTC_TimeTypeDef timep;
	RTC_DateTypeDef datep;
	// open file and write the beginning of the load
	HAL_RTC_GetTime(&hrtc, &timep, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &datep, RTC_FORMAT_BIN);
	//rtcGetTime(&RTCD1, &timep);
	sprintf(sLine, "%04d-%02d-%02dT%02d-%02d-%02dZ.csv", datep.Year + 1900,
			datep.Month, datep.Date, timep.Hours, timep.Minutes, timep.Seconds); // making new file

	logfile = fopen_(sLine, "a");
	if (bIncludeTimestamp)
		strcpy(sLine,
				"Timestamp,ID,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7\r\n");
	else
		strcpy(sLine, "ID,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7\r\n");
	writeToSDBuffer(sLine);
	align_buffer();
	fwrite_(sd_buffer, 1, sd_buffer_length, &logfile);
	f_sync(logfile);

	// reset buffer counters
	sd_buffer_length_for_write = 0;
	sd_buffer_length = 0;

	bWriteFault = 0;

	stLastWriting = HAL_GetTick(); // record time when we did write

	bLogging = 1;
}

/**
 * @brief Returns Prescaler, BS1 and BS2 settings for CAN calculated from bitrate
 * @param bitrate
 * @return CAN_BitrateSetting | default returns settings for 500 kbps
 */
int get_CAN_setBaudeRate(uint32_t bitrate) {
	for (int var = 0; var < 8; ++var) {
		if (bitrate == CAN_BitrateSettingsArray[var].bitrate) {
			return var;
		}
	}
	return 0;
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

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE
			| RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
			{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
			{
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
			{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* CAN init function */
static void MX_CAN_Init(void) {

	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 16;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SJW = CAN_SJW_1TQ;
	hcan.Init.BS1 = CAN_BS1_1TQ;
	hcan.Init.BS2 = CAN_BS2_1TQ;
	hcan.Init.TTCM = DISABLE;
	hcan.Init.ABOM = DISABLE;
	hcan.Init.AWUM = DISABLE;
	hcan.Init.NART = DISABLE;
	hcan.Init.RFLM = DISABLE;
	hcan.Init.TXFP = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* I2C2 init function */
static void MX_I2C2_Init(void) {

	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* SPI2 init function */
static void MX_SPI2_Init(void) {

	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void) {

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USB init function */
static void MX_USB_PCD_Init(void) {

	hpcd_USB_FS.Instance = USB;
	hpcd_USB_FS.Init.dev_endpoints = 8;
	hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_FS.Init.ep0_mps = DEP0CTL_MPS_8;
	hpcd_USB_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* RTC init function */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef DateToUpdate;

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/**Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

	/**Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	/* USER CODE BEGIN RTC_Init 3 */

	/* USER CODE END RTC_Init 3 */

	DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
	DateToUpdate.Month = RTC_MONTH_JANUARY;
	DateToUpdate.Date = 0x1;
	DateToUpdate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	/* USER CODE BEGIN RTC_Init 4 */

	/* USER CODE END RTC_Init 4 */

}

/* CRC init function */
static void MX_CRC_Init(void) {

	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
				;
	__HAL_RCC_GPIOD_CLK_ENABLE()
				;
	__HAL_RCC_GPIOA_CLK_ENABLE()
				;
	__HAL_RCC_GPIOB_CLK_ENABLE()
				;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CAN_CTRL_GPIO_Port, CAN_CTRL_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED2_Pin | LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(MMC_CS_GPIO_Port, MMC_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CAN_CTRL_Pin */
	GPIO_InitStruct.Pin = CAN_CTRL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CAN_CTRL_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED2_Pin LED1_Pin */
	GPIO_InitStruct.Pin = LED2_Pin | LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : BUT_Pin USB_P_Pin */
	GPIO_InitStruct.Pin = BUT_Pin | USB_P_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : MMC_CS_Pin */
	GPIO_InitStruct.Pin = MMC_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MMC_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument) {

	/* USER CODE BEGIN 5 */
	  /* init code for FATFS */
	  //MX_FATFS_Init();

	  /* init code for USB_DEVICE */
#ifdef USE_USB
	  MX_USB_DEVICE_Init();
#endif
	/* Infinite loop */
	for (;;) {
		sdcard_systick_timerproc();
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
