/*
 * serial.h
 * function prototypes for serial
 */

#ifndef SERIAL_H_
#define SERIAL_H_

void serialPrint(const char * str);
uint16_t serialRead(char * buffer, uint16_t size, char stopChar);

#endif
