/*
 * uart.h
 * function prptotypes for uart
 */

#ifndef UART_H_
#define UART_H_

void serialPrint(const char * str);
uint16_t serialRead(char * buffer, uint16_t size, char stopChar);

#endif
