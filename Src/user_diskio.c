/**
 ******************************************************************************
 * @file    user_diskio.c
 * @brief   This file includes a diskio driver skeleton to be completed by the user.
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

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/* 
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future. 
 * Kept to ensure backward compatibility with previous CubeMx versions when 
 * migrating projects. 
 * User code previously added there should be copied in the new user sections before 
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Definitions for MMC/SDC command */
#define CMD0     (0x40+0)     /* GO_IDLE_STATE */
#define CMD1     (0x40+1)     /* SEND_OP_COND */
#define CMD8     (0x40+8)     /* SEND_IF_COND */
#define CMD9     (0x40+9)     /* SEND_CSD */
#define CMD10    (0x40+10)    /* SEND_CID */
#define CMD12    (0x40+12)    /* STOP_TRANSMISSION */
#define CMD16    (0x40+16)    /* SET_BLOCKLEN */
#define CMD17    (0x40+17)    /* READ_SINGLE_BLOCK */
#define CMD18    (0x40+18)    /* READ_MULTIPLE_BLOCK */
#define CMD23    (0x40+23)    /* SET_BLOCK_COUNT */
#define CMD24    (0x40+24)    /* WRITE_BLOCK */
#define CMD25    (0x40+25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD41    (0x40+41)    /* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)    /* APP_CMD */
#define CMD58    (0x40+58)    /* READ_OCR */
/* Private variables ---------------------------------------------------------*/
static volatile DSTATUS Stat = STA_NOINIT; /* Disk status */
static volatile BYTE Timer1, Timer2; /* 100Hz decrement timer */
static BYTE CardType; /* b0:MMC, b1:SDC, b2:Block addressing */
static BYTE PowerFlag = 0; /* indicates if "power" is on */
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize(BYTE pdrv);
DSTATUS USER_status(BYTE pdrv);
DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef USER_Driver = { USER_initialize, USER_status, USER_read,
#if  _USE_WRITE
		USER_write,
#endif  /* _USE_WRITE == 1 */  
#if  _USE_IOCTL == 1
		USER_ioctl,
#endif /* _USE_IOCTL == 1 */
		};

/* Private functions ---------------------------------------------------------*/
/**
 * @brief Pulls down chip select pin for SD card and selects card
 */
static
void SELECT(void) {
	HAL_GPIO_WritePin(MMC_CS_GPIO_Port, MMC_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Pulls up chip select pin for SD card and deselects it
 */
static
void DESELECT(void) {
	HAL_GPIO_WritePin(MMC_CS_GPIO_Port, MMC_CS_Pin, GPIO_PIN_SET);
}

static
void xmit_spi(BYTE Data) {
	while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
		;
	HAL_SPI_Transmit(&hspi2, &Data, 1, 5000);
}

static BYTE rcvr_spi(void) {
	unsigned char Dummy, Data;
	Dummy = 0xFF;
	Data = 0;
	while ((HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY))
		;
	HAL_SPI_TransmitReceive(&hspi2, &Dummy, &Data, 1, 5000);

	return Data;
}

static void rcvr_spi_m(BYTE *dst) {
	*dst = rcvr_spi();
}

/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------z------------------------------------------*/

/**
 * @brief Wait for card to be ready
 * @return BYTE res
 */
static BYTE wait_ready(void) {
	BYTE res;

	Timer2 = 50;
	rcvr_spi();
	do
		res = rcvr_spi();
	while ((res != 0xFF) && Timer2);

	return res;
}


/**
 *@brief Power Control  (Platform dependent)
 *@brief When the target system does not support socket power control, there
 *@brief is nothing to do in these functions and chk_power always returns 1.
 */
static void power_on(void) {
	unsigned char i, cmd_arg[6];
	unsigned int Count = 0x1FFF;

	DESELECT();

	for (i = 0; i < 10; i++)
		xmit_spi(0xFF);

	SELECT();

	cmd_arg[0] = (CMD0 | 0x40);
	cmd_arg[1] = 0;
	cmd_arg[2] = 0;
	cmd_arg[3] = 0;
	cmd_arg[4] = 0;
	cmd_arg[5] = 0x95;

	for (i = 0; i < 6; i++)
		xmit_spi(cmd_arg[i]);

	while ((rcvr_spi() != 0x01) && Count)
		Count--;

	DESELECT();
	xmit_spi(0XFF);

	PowerFlag = 1;
}


static void power_off(void) {
	PowerFlag = 0;
}

/**
 * @brief Socket power state: 0=off, 1=on
 * @return PowerFlag
 */
static int chk_power(void)
{
	return PowerFlag;
}

/**
 * @brief Receive a data packet from MMC
 * @param buff Data buffer to store received data
 * @param btr Byte count (must be even number)
 * @return true
 */
static bool rcvr_datablock(BYTE *buff, UINT btr ) {
	BYTE token;

	Timer1 = 10;
	do { /* Wait for data packet in timeout of 100ms */
		token = rcvr_spi();
	} while ((token == 0xFF) && Timer1);
	if (token != 0xFE)
		return false; /* If not valid data token, return with error */

	do { /* Receive the data block into buffer */
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
	} while (btr -= 2);
	rcvr_spi(); /* Discard CRC */
	rcvr_spi();

	return true; /* Return with success */
}

/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
/**
 * @brief Send a data packet to MMC
 * @param buff 512 byte data block to be transmitted
 * @param token Data/Stop token
 * @return
 */
static bool xmit_datablock(const BYTE *buff, BYTE token) {
	BYTE resp, wc;
	uint32_t i = 0;

	if (wait_ready() != 0xFF)
		return false;

	xmit_spi(token); /* Xmit data token */
	if (token != 0xFD) { /* Is data token */
		wc = 0;
		do { /* Xmit the 512 byte data block to MMC */
			xmit_spi(*buff++);
			xmit_spi(*buff++);
		} while (--wc);

		rcvr_spi();
		rcvr_spi();

		while (i <= 64) {
			resp = rcvr_spi(); /* Reveive data response */
			if ((resp & 0x1F) == 0x05) /* If not accepted, return with error */
				break;
			i++;
		}
		while (rcvr_spi() == 0)
			;
	}
	if ((resp & 0x1F) == 0x05)
		return true;
	else
		return false;
}
#endif /* _READONLY */

/**
 * @brief Send a command packet to MMC
 * @param cmd Command byte
 * @param arg Argument
 * @return
 */
static BYTE send_cmd(BYTE cmd, DWORD arg) {
	BYTE n, res;

	if (wait_ready() != 0xFF)
		return 0xFF;

	/* Send command packet */
	xmit_spi(cmd); /* Command */
	xmit_spi((BYTE) (arg >> 24)); /* Argument[31..24] */
	xmit_spi((BYTE) (arg >> 16)); /* Argument[23..16] */
	xmit_spi((BYTE) (arg >> 8)); /* Argument[15..8] */
	xmit_spi((BYTE) arg); /* Argument[7..0] */
	n = 0;
	if (cmd == CMD0)
		n = 0x95; /* CRC for CMD0(0) */
	if (cmd == CMD8)
		n = 0x87; /* CRC for CMD8(0x1AA) */
	xmit_spi(n);

	/* Receive command response */
	if (cmd == CMD12)
		rcvr_spi(); /* Skip a stuff byte when stop reading */
	n = 10; /* Wait for a valid response in timeout of 10 attempts */
	do
		res = rcvr_spi();
	while ((res & 0x80) && --n);

	return res; /* Return with the response value */
}

/*-----------------------------------------------------------------------*/
/* FATFS USER FUNCTIONS                                                  */
/* Functions below were generated by CubeMX but the logic needs to be    */
/* input by user                                                         */
/*-----------------------------------------------------------------------*/

/**
 * @brief  Initializes a Drive
 * @param  pdrv: Physical drive number (0..)
 * @retval DSTATUS: Operation status
 */
DSTATUS USER_initialize(BYTE pdrv /* Physical drive nmuber to identify the drive */
) {
	/* USER CODE BEGIN INIT */
	Stat = STA_NOINIT;
	return Stat;
	/* USER CODE END INIT */
}

/**
 * @brief  Gets Disk Status
 * @param  pdrv: Physical drive number (0..)
 * @retval DSTATUS: Operation status
 */
DSTATUS USER_status(BYTE pdrv /* Physical drive number to identify the drive */
) {
	/* USER CODE BEGIN STATUS */
	Stat = STA_NOINIT;
	return Stat;
	/* USER CODE END STATUS */
}

/**
 * @brief  Reads Sector(s)
 * @param  pdrv: Physical drive number (0..)
 * @param  *buff: Data buffer to store read data
 * @param  sector: Sector address (LBA)
 * @param  count: Number of sectors to read (1..128)
 * @retval DRESULT: Operation result
 */
DRESULT USER_read(BYTE pdrv, /* Physical drive nmuber to identify the drive */
BYTE *buff, /* Data buffer to store read data */
DWORD sector, /* Sector address in LBA */
UINT count /* Number of sectors to read */
) {
	/* USER CODE BEGIN READ */
	return RES_OK;
	/* USER CODE END READ */
}

/**
 * @brief  Writes Sector(s)
 * @param  pdrv: Physical drive number (0..)
 * @param  *buff: Data to be written
 * @param  sector: Sector address (LBA)
 * @param  count: Number of sectors to write (1..128)
 * @retval DRESULT: Operation result
 */
#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, /* Physical drive nmuber to identify the drive */
const BYTE *buff, /* Data to be written */
DWORD sector, /* Sector address in LBA */
UINT count /* Number of sectors to write */
) {
	/* USER CODE BEGIN WRITE */
	/* USER CODE HERE */
	return RES_OK;
	/* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
 * @brief  I/O control operation
 * @param  pdrv: Physical drive number (0..)
 * @param  cmd: Control code
 * @param  *buff: Buffer to send/receive control data
 * @retval DRESULT: Operation result
 */
#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, /* Physical drive nmuber (0..) */
BYTE cmd, /* Control code */
void *buff /* Buffer to send/receive control data */
) {
	/* USER CODE BEGIN IOCTL */
	DRESULT res = RES_ERROR;
	return res;
	/* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
