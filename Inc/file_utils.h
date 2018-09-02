/*
 * file_utils.h
 *
 *  Created on: Sep 2, 2018
 *      Author: tdarlic
 */

#ifndef INC_FILE_UTILS_H_
#define INC_FILE_UTILS_H_

FIL * fopen_(const char * fileName, const char *mode);
int fclose_(FIL *file);
size_t fwrite_(const void *data_to_write, size_t size, size_t n, FIL *stream);
size_t fread_(void *ptr, size_t size, size_t n, FIL *stream);
bool finit_(void);

#endif /* INC_FILE_UTILS_H_ */
