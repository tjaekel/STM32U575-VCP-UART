/**
 * VCP_UART.h
 *
 *  Created on: Jul 18, 2022
 *      Author: Torsten Jaekel
 */

#ifndef INC_VCP_UART_H_
#define INC_VCP_UART_H_

#include <stdint.h>
#include <stdio.h>

#include "usbd_cdc_if.h"

#define UART_RX_BUFFER_SIZE	4096	//1024
#define UART_TX_BUFFER_SIZE 4096	//1024

typedef enum {
	VCP_OUT,
	UART_OUT,
	SILENT
} EResultOut;

extern unsigned char uartPrintBuffer[UART_RX_BUFFER_SIZE];

int UART_getChar(void);
int UART_WaitForChar(void);
void UART_putChar(unsigned char c);
int UART_getString(unsigned char *buf, int maxLen);
void UART_Send(unsigned char *buf, int len, EResultOut out);
void UART_printString(unsigned char *str, EResultOut out);
void UART_setLastString(unsigned char *last);

#endif /* INC_VCP_UART_H_ */
