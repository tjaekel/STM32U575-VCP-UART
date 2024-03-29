/*
 * QSPI.h
 *
 *  Created on: Dec 22, 2023
 *      Author: tj
 */

#ifndef INC_QSPI_H_
#define INC_QSPI_H_

#include "main.h"

extern OSPI_HandleTypeDef hospi1;

uint8_t QSPI_Transaction(unsigned long *params, unsigned long numParams, unsigned long rdWords);
unsigned long QSPI_SetClock(unsigned long div);
unsigned long QSPI_ReadChipID(void);

#endif /* INC_QSPI_H_ */
