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
#include <stdio.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "fatfs.h"
#include <string.h>
#include "file_utils.h"
#include "stm32f1xx_it.h"
#ifdef USE_USB
#include "usb_device.h"
#endif

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
// Definition of CAN interface and the messages
CAN_HandleTypeDef hcan;
CanRxMsgTypeDef hcanRxMsg1;
CanRxMsgTypeDef hcanRxMsg2;

CanTxMsgTypeDef canTxMsg;
CRC_HandleTypeDef hcrc;
I2C_HandleTypeDef hi2c2;
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
#ifdef USE_USB
PCD_HandleTypeDef hpcd_USB_FS;
#endif
osThreadId defaultTaskHandle;

osThreadId blinkLed1TID;
osThreadId blinkLed2TID;
osThreadId logCANBusTID;
osThreadId buttonDebounceTID;

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
const CAN_BitrateSetting CAN_BitrateSettingsArray72[8] = {
		//brp, pre, BS1, BS2
		{ 500, 2, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 250, 18, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 125, 36, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 100, 45, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 83, 54, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 50, 90, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 20, 225, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 10, 450, CAN_BS1_13TQ, CAN_BS2_2TQ }
};

/**
 * Calculations taken from webpage:
 * http://www.bittiming.can-wiki.info/
 * With following settings
 * Type: bxCAN, Clock: 36MHz, max brp: 1024,
 * SP: 87.5%, min tq: 8, max tq: 25, FD factor: undefined, SJW: 1
 */
const CAN_BitrateSetting CAN_BitrateSettingsArray[8] = {
		//brp, pre, BS1, BS2
		{ 500, 9, CAN_BS1_6TQ, CAN_BS2_1TQ },
		//{ 250, 18, CAN_BS1_13TQ, CAN_BS2_2TQ },
		//http://old.ghielectronics.com/community/forum/topic?id=1504
		//T1 = 15, T2 = 8 will give us 15 + 8 + 1 = 24 and this is what we need
		{ 250, 6, CAN_BS1_15TQ, CAN_BS2_8TQ }, //<-- operational
		{ 125, 36, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 100, 45, CAN_BS1_15TQ, CAN_BS2_2TQ },
		{ 83, 54, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 50, 90, CAN_BS1_13TQ, CAN_BS2_2TQ },
		{ 20, 225, CAN_BS1_15TQ, CAN_BS2_2TQ },
		{ 10, 450, CAN_BS1_13TQ, CAN_BS2_2TQ }
};

// @formatter:on

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define STRLINE_LENGTH 					1024
#define SD_WRITE_BUFFER             	(1024*2)// 3k
#define SD_WRITE_BUFFER_FLUSH_LIMIT 	(1024*1)// 2k
#define MMCSD_BLOCK_SIZE 				512

char sLine[STRLINE_LENGTH];

FATFS g_sFatFs;
FIL* logfile;

uint32_t stLastWriting;

osSemaphoreId canRcvLedSemId;

unsigned char bLogging = 0; // if =1 than we logging to SD card

// Signal to be sent to the LED1 thread when button is pressed
const int32_t buttonPressed = 1;
// Signal to be sent to the LED2 when CAN frame is received
const int32_t canReceived = 2;

int iFilterMask = 0;
int iFilterValue = 0;
uint8_t bLogStdMsgs = 1;
uint8_t bLogExtMsgs = 1;
uint8_t bIncludeTimestamp = 1;

// buffer for collecting data to write
BYTE sd_buffer[SD_WRITE_BUFFER];
uint16_t sd_buffer_length = 0;

// buffer for storing ready to write data
BYTE sd_buffer_for_write[SD_WRITE_BUFFER];
unsigned char bReqWrite = 0; // write request, the sd_buffer is being copied to sd_buffer_for_write
WORD sd_buffer_length_for_write = 0;

// Mail message queue for the CAN messages from interrupt to CAN logger
// Queue to hold 5 messages
// https://www.keil.com/pack/doc/cmsis/RTOS/html/group__CMSIS__RTOS__Mail.html
osMailQId canMsgBoxHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C2_Init(void);
#ifdef USE_USB
static void MX_USB_PCD_Init(void);
#endif
static void MX_RTC_Init(void);
static void MX_CRC_Init(void);
void canTxMessage(void);
void StartDefaultTask(void const * argument);
void blinkLed1Thread(void const *argument);
void blinkLed2Thread(void const *argument);
void logCANBusThread(void const *argument);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);
void buttonDebounceThread(void const *argument);
int get_CAN_setBaudeRate(uint32_t bitrate);
static FRESULT mountSDCard(void);
static int read_config_file(void);
static int align_buffer(void);
static void copy_buffer(void);
static void request_write(void);
static void writeToSDBuffer(char *pString);
void testWriteToSD(void);
void startLog(void);
void stopLog(void);
extern void initialise_monitor_handles(void); /* prototype */
void threadsDump(void);

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
#ifdef USE_USB
	MX_USB_PCD_Init();
#endif
	MX_RTC_Init();
	MX_CRC_Init();
	MX_NVIC_Init();

	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(CAN_CTRL_GPIO_Port, CAN_CTRL_Pin, GPIO_PIN_RESET);

	//switch off all LEDS
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

	/* init code for FATFS */
	MX_FATFS_Init();
	mountSDCard();
	if (!read_config_file()) {
		// config file was not read, just flash the led
		for (;;) {
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			HAL_Delay(30);
		}
	}

	//enable semihosting
	//https://mcuoneclipse.com/2014/09/11/semihosting-with-gnu-arm-embedded-launchpad-and-gnu-arm-eclipse-debug-plugins/
	initialise_monitor_handles();
#ifdef DEBUG
	printf("SWO Debug out enabled \r\n");
#endif
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	//osSemaphoreDef(canRcvLedSem);
	//canRcvLedSemId = osSemaphoreCreate(osSemaphore(canRcvLedSem), 1);
	//osSemaphoreWait(canRcvLedSemId, osWaitForever);
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */

	//Blink LED1 thread is not used
	//osThreadDef(blinkLed1, blinkLed1Thread, osPriorityNormal, 0, 100);
	//blinkLed1TID = osThreadCreate(osThread(blinkLed1), NULL);
	osThreadDef(blinkLed2, blinkLed2Thread, osPriorityNormal, 0, 128);
	blinkLed2TID = osThreadCreate(osThread(blinkLed2), NULL);

	osThreadDef(logCANBus, logCANBusThread, osPriorityNormal, 0, 256);
	logCANBusTID = osThreadCreate(osThread(logCANBus), NULL);

	osThreadDef(buttonDebounce, buttonDebounceThread, osPriorityNormal, 0, 256);
	buttonDebounceTID = osThreadCreate(osThread(buttonDebounce), NULL);
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	// Create mail queue for the CAN messages
	osMailQDef(canMsgBox, 5, CanRxMsgTypeDef);
	canMsgBoxHandle = osMailCreate(osMailQ(canMsgBox), NULL);
	/* USER CODE END RTOS_QUEUES */

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

/**
 * Dumping threads status, this is used only if debugging
 */
void threadsDump(void) {
	TaskStatus_t *pxTaskStatusArray = NULL;
	char *pcBuf = NULL;
	char *pcStatus;

	/* Allocate the message buffer. */
	pcBuf = pvPortMalloc(100 * sizeof(char));

	/* Allocate an array index for each task. */
	pxTaskStatusArray = pvPortMalloc(
			uxTaskGetNumberOfTasks() * sizeof(TaskStatus_t));

	if (pcBuf != NULL && pxTaskStatusArray != NULL) {
		/* Generate the (binary) data. */
		uxTaskGetSystemState(pxTaskStatusArray, uxTaskGetNumberOfTasks(), NULL);

		sprintf(pcBuf,
				"         LIST OF RUNNING THREADS         \r\n-----------------------------------------\r\n");
		printf(pcBuf);

		for (uint16_t i = 0; i < uxTaskGetNumberOfTasks(); i++)
				{
			sprintf(pcBuf, "Thread: %s\r\n",
					pxTaskStatusArray[i].pcTaskName);
			printf(pcBuf);

			sprintf(pcBuf, "Thread ID: %lu\r\n",
					pxTaskStatusArray[i].xTaskNumber);
			printf(pcBuf);

			switch (pxTaskStatusArray[i].eCurrentState) {
			case eReady:
				pcStatus = "READY";
				break;
			case eBlocked:
				pcStatus = "BLOCKED";
				break;
			case eSuspended:
				pcStatus = "SUSPENDED";
				break;
			case eDeleted:
				pcStatus = "DELETED";
				break;

			default: /* Should not get here, but it is included
			 to prevent static checking errors. */
				pcStatus = 0x00;
				break;
			}

			sprintf(pcBuf, "\tStatus: %s\r\n", pcStatus);
			printf(pcBuf);

			sprintf(pcBuf, "\tStack watermark number: %d\r\n",
					pxTaskStatusArray[i].usStackHighWaterMark);
			printf(pcBuf);

			sprintf(pcBuf, "\tPriority: %lu\r\n",
					pxTaskStatusArray[i].uxCurrentPriority);
			printf(pcBuf);

			sprintf(pcBuf, "\tRun-time time: %lu\r\n",
					pxTaskStatusArray[i].ulRunTimeCounter);
			printf(pcBuf);

			sprintf(pcBuf, "\tRun-time time in percentage: %lu\r\n",
					pxTaskStatusArray[i].ulRunTimeCounter
							/ portGET_RUN_TIME_COUNTER_VALUE() / 100);
			printf(pcBuf);
		}

		vPortFree(pcBuf);
		vPortFree(pxTaskStatusArray);
	}

	//osThreadTerminate(NULL);
}

void canTxMessage(void) {
	uint8_t Data[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	HAL_StatusTypeDef status;
	canTxMsg.DLC = 1;
	for (int var = 0; var < 8; ++var) {
		canTxMsg.Data[var] = Data[var];
	}
	canTxMsg.DLC = 8;
	canTxMsg.ExtId = 0x02;
	canTxMsg.IDE = CAN_ID_STD;
	canTxMsg.StdId = 0x01;
	hcan.pTxMsg = &canTxMsg;
	status = HAL_CAN_Transmit(&hcan, 100);
	if (status == HAL_ERROR) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

/**
 * @brief GPIO button interrupt callback routine
 * @param GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == BUT_Pin) {
		osSignalSet(buttonDebounceTID, buttonPressed);
		// Make sure that the values are written to the memory before exiting
		// interrupt procedure because the memory command is still in the buffer
		// and it maybe was not written to the memory yet so interrupt is still
		// active
		// http://www.keil.com/support/docs/3928.htm
		__DSB();
	}
}

/**
 * @brief CAN interrupt callback routine
 * @param hcan
 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan) {
	CanRxMsgTypeDef *canMsg;
	//uint32_t canMsg;
	//CAN message received - put it in mail and sent to thread to write
	// if not logging then return
	if (!bLogging) {
		return;
	}

	osSignalSet(blinkLed2TID, canReceived);
	canMsg = (CanRxMsgTypeDef *) osMailAlloc(canMsgBoxHandle, osWaitForever);
	// copy values from CAN FIFO to canMsg so it is not overwritten
	*canMsg = *(hcan->pRxMsg);

	osMailPut(canMsgBoxHandle, canMsg);

	//rearm the interrupt for CAN
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
	//make sure all memory access is completed before exiting this function
	__DSB();

}

void buttonDebounceThread(void const *argument) {
	GPIO_PinState ledState = GPIO_PIN_RESET;
	while (1) {
		osSignalWait(buttonPressed, osWaitForever);
		osDelay(100);
		if (HAL_GPIO_ReadPin(BUT_GPIO_Port, BUT_Pin) == GPIO_PIN_SET) {
			bLogging = !bLogging;
			if (bLogging) {
				ledState = GPIO_PIN_SET;
				startLog();
				//canTxMessage();
			} else {
				ledState = GPIO_PIN_RESET;
				stopLog();
			}
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, ledState);
		}
#ifdef DEBUG
		threadsDump();
#endif
	}
	osThreadTerminate(NULL);
}

void blinkLed1Thread(void const *argument) {
	while (1) {
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		osDelay(1000);
	}
	osThreadTerminate(NULL);
}

void blinkLed2Thread(void const *argument) {
	uint8_t i;

	while (1) {
		osSignalWait(canReceived, osWaitForever);
		for (i = 0; i < 12; ++i) {
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			osDelay(70);
		}
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	}
	osThreadTerminate(NULL);
}

void logCANBusThread(void const *argument) {
	CanRxMsgTypeDef *canMsg;
	uint8_t i;
	char sTmp[128];

	while (1) {
		osEvent canEvent = osMailGet(canMsgBoxHandle, osWaitForever);
		canMsg = (CanRxMsgTypeDef *) canEvent.value.p; // ".p" indicates that the message is a pointer
		// message received do the magic and write to SD
		// write down data
		if (bIncludeTimestamp)
			sprintf(sTmp, "%d,%X", osKernelSysTick(), canMsg->ExtId);
		else
			sprintf(sTmp, "%X", canMsg->ExtId);

		for (i = 0; i < canMsg->DLC; i++)
				{
			sprintf(sTmp + strlen(sTmp), ",%02X", canMsg->Data[i]);
		}

		strcat(sTmp, "\r\n");
		writeToSDBuffer(sTmp);
		osMailFree(canMsgBoxHandle, canMsg);
	}
	osThreadTerminate(NULL);
}

/**
 * If debugging break when stack overflow is detected
 * @param xTask
 * @param pcTaskName
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
	asm("BKPT #0");
}
//char createStrFrommCanMsg()

static FRESULT mountSDCard(void) {
	return f_mount(&g_sFatFs, "0:", 0);
}

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
	for (i = 0; i < len; i++) {
		sd_buffer[sd_buffer_length + i - 2] = ' ';
	}
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
	static FRESULT fresult;
	uint32_t bytes_written = 0;

	if (bReqWrite)
		bWriteFault = 1; // buffer overlapping

	// carry out write operation
	align_buffer();
	copy_buffer();

	uint32_t blen = 1023;

	//go to the end of the file
	//fresult = f_lseek(&logfile, logfile->fsize);

	bytes_written = fwrite_(sd_buffer_for_write, 1, sd_buffer_length_for_write,
			logfile);

	if (bytes_written != sd_buffer_length_for_write) {
		bWriteFault = 2;
	}

	if (f_sync(logfile) != FR_OK) {
		bWriteFault = 2;
	}
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

	// record time when we did write
	stLastWriting = HAL_GetTick();

	// Check flush limit
	if (sd_buffer_length >= SD_WRITE_BUFFER_FLUSH_LIMIT)
	{
		request_write();
	}
}

/**
 * @brief Test function used successfully to write to Sd card
 */
void testWriteToSD(void) {
	char buffer[128];

	static FRESULT fresult;
	FIL file;

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
	sprintf(buffer, "Hello World!\r\n");

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
	int baud = 500;
	int res = 0;
	int ack = 0;

	iFilterMask = 0;
	iFilterValue = 0;
	bIncludeTimestamp = 1;
	bLogStdMsgs = 1;
	bLogExtMsgs = 1;

	uint8_t brs = 0;

	//fresult = f_mount(&g_sFatFs, "0:", 0);

	//open file on SD card
	fresult = f_open(&file, "0:Config.txt", FA_OPEN_EXISTING | FA_READ);
	// if result is not FR_OK then there was an error opening
	if (fresult != FR_OK) {
		return 0;
		//todo: Add flashing LED or not turning LED on on error
	}

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
			bIncludeTimestamp = (uint8_t) value;
		} else if (strcmp(name, "log_std") == 0) {
			bLogStdMsgs = (uint8_t) value;
		} else if (strcmp(name, "log_ext") == 0) {
			bLogExtMsgs = (uint8_t) value;
		}
	}
	// close SD file
	f_close(&file);

	// configure CAN
	brs = (uint8_t) get_CAN_setBaudeRate(baud);

	HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

	if (HAL_CAN_DeInit(&hcan) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	hcan.Init.Prescaler = CAN_BitrateSettingsArray[brs].Prescaler;
	hcan.Init.BS1 = CAN_BitrateSettingsArray[brs].BS1;
	hcan.Init.BS2 = CAN_BitrateSettingsArray[brs].BS2;
	if (ack) {
		hcan.Init.Mode = CAN_MODE_NORMAL;
	} else {
		hcan.Init.Mode = CAN_MODE_NORMAL; //CAN_MODE_SILENT;
	}

	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	// start the CAN interrupt
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

	HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

	HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

	__HAL_CAN_CLEAR_FLAG(&hcan, CAN_IT_FMP0);
	__HAL_CAN_ENABLE_IT(&hcan, CAN_IT_FMP0);

	return res;
}

/**
 * @brief Stops the logging and closes the file on disk
 */
void stopLog() {
	__HAL_CAN_DISABLE_IT(&hcan, CAN_IT_FMP0);
	fclose_(sLine);
	// reset buffer counters
	sd_buffer_length_for_write = 0;
	sd_buffer_length = 0;

	bWriteFault = 0;

	stLastWriting = HAL_GetTick(); // record time when we did write

	bLogging = 0;
}

/**
 * @brief Prepare file on SD card for writing to and write header
 * 		  after that set the bLogging variable to 1 so that the
 *		  thread writing to SD continues to write
 */
void startLog(void)
{

	RTC_TimeTypeDef timep;
	// reset buffer counters
	sd_buffer_length_for_write = 0;
	sd_buffer_length = 0;

	bWriteFault = 0;

	//RTC_DateTypeDef datep;
	// open file and write the beginning of the load
	HAL_RTC_GetTime(&hrtc, &timep, RTC_FORMAT_BIN);
	//HAL_RTC_GetDate(&hrtc, &datep, RTC_FORMAT_BIN);
	//rtcGetTime(&RTCD1, &timep);
	sprintf(sLine, "0:%02d%02d%02d.csv", timep.Hours, timep.Minutes,
			timep.Seconds); // making new file

	logfile = fopen_(sLine, "w");
	if (bIncludeTimestamp)
		strcpy(sLine,
				"Timestamp,ID,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7\r\n");
	else
		strcpy(sLine, "ID,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7\r\n");
	writeToSDBuffer(sLine);
	//align_buffer();
	//fwrite_(sd_buffer, 1, sd_buffer_length, logfile);
	//f_sync(logfile);
	// setup CAN bus interrupt
	__HAL_CAN_ENABLE_IT(&hcan, CAN_IT_FMP0);

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

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void)
{
	HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
	/* EXTI9_5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* CAN init function */
static void MX_CAN_Init(void) {

	CAN_FilterConfTypeDef sFilterConfig;

	hcan.pRxMsg = &hcanRxMsg1;
	hcan.pRx1Msg = &hcanRxMsg2;

	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 2;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SJW = CAN_SJW_1TQ;
	hcan.Init.BS1 = CAN_BS1_6TQ;
	hcan.Init.BS2 = CAN_BS2_8TQ;
	hcan.Init.TTCM = DISABLE;
	hcan.Init.ABOM = DISABLE;
	hcan.Init.AWUM = DISABLE;
	hcan.Init.NART = DISABLE;
	hcan.Init.RFLM = DISABLE;
	hcan.Init.TXFP = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
			{
		/* Filter configuration Error */
		Error_Handler();
	}

	__HAL_CAN_DBG_FREEZE(&hcan, DISABLE);
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

#ifdef USE_USB
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
#endif

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
	// enable CAN clock
	__HAL_RCC_CAN1_CLK_ENABLE()
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

	/*Configure GPIO pin : BUT_Pin */
	GPIO_InitStruct.Pin = BUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUT_GPIO_Port, &GPIO_InitStruct);

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
