/*
 * rc.h
 * function prototypes for rc
 */

#ifndef UART_H_
#define UART_H_

#define RC_LOW_LIM 500
#define RC_HIGH_LIM 2500
#define CH0_MIN 1033
#define CH0_MAX 1873
#define CH1_MIN 1032
#define CH1_MAX 1850
#define GEAR_MIN 1030
#define GEAR_MAX 1870

void rcInit(void);
uint8_t checkPulse(uint16_t pw);
int32_t map(int32_t x, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax);
int32_t bound(int32_t val, int32_t lowLimit, int32_t highLimit);

#endif
