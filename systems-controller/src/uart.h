/*
 * uart.h
 * function prptotypes for uart
 */

#ifndef UART_H_
#define UART_H_

#define BAUD2DIV(baud)  (((F_CPU * 2) + ((baud) >> 1)) / (baud))

void uartInit(uint32_t baudrate);
void uartSendChar(char data);
char uartGetChar(void);
void uartPrint(const char * data);

#endif
