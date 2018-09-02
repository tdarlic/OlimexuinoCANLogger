/*
 * file_utils.c
 *
 *  Created on: Sep 2, 2018
 *      Author: tdarlic
 */

/*===========================================================================*/
// functions to convert the FatFs functions to C standard routines
#include "ff.h"
#include <stdbool.h>

static FIL file_sdc;

//extern bool_t fs_ready;

/**
 * Open the file from SD card
 * @param fileName String containing the filename
 * @param mode 'r' for read, 'w' for write and 'a' for append
 * @return FIL File or 0 on error
 */
FIL * fopen_(const char * fileName, const char *mode)
{
	FRESULT fres;
	BYTE attr = FA_READ;
	FIL* File = &file_sdc;

	if (mode[0] == 'a')
			{
		fres = f_open(File, fileName, FA_WRITE | FA_OPEN_EXISTING);
		if (fres != FR_OK)
				{
			// file does not exist, create it
			fres = f_open(File, fileName, FA_WRITE | FA_OPEN_ALWAYS);
		}
		else
		{
			// rewind file to end for append write
			fres = f_lseek(File, f_size(File));
		}
	}
	else
	{
		// determine attributes to open file
		if (mode[0] == 'r')
			attr = FA_READ;
		else
		if (mode[0] == 'w')
			attr = FA_WRITE | FA_OPEN_ALWAYS;

		fres = f_open(File, fileName, attr);
	}

	// if file opened -- return pointer to local variable
	if (fres == FR_OK)
		return File;
	else
		return 0;
}

/**
 * Closes the file on SD card
 * @param Pointer to file
 * @return
 */
int fclose_(FIL *file)
{
	//TODO: f_sync ?

	if (f_close(file) == FR_OK)
		return 0;
	else
		return EOF;
}

size_t fwrite_(const void *data_to_write, size_t size, size_t n, FIL *stream)
{
	UINT data_written;
	FRESULT fres;

	fres = f_write(stream, data_to_write, size * n, &data_written);
	if (fres != FR_OK)
		return 0;

	return data_written;
}

size_t fread_(void *ptr, size_t size, size_t n, FIL *stream)
{
	UINT data_written;
	FRESULT fres;

	fres = f_read(stream, ptr, size * n, &data_written);
	if (fres != FR_OK)
		return 0;

	return data_written;
}

bool finit_(void)
{
	/*
	 * Activates the SDC driver 1 using default configuration.
	 */

	return true;
}

