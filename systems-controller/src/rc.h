/*
 * rc.h
 * function prototypes for rc
 */

#ifndef UART_H_
#define UART_H_

#define RC_THRESH 100
#define RC_LOW_LIM 500
#define RC_MID 1500
#define RC_HIGH_LIM 2500
#define CH0_MIN 1033
#define CH0_MAX 1873
#define CH1_MIN 1032
#define CH1_MAX 1850
#define ESTOP_MIN 1030
#define ESTOP_MAX 1870
#define MODE_MIN 1030
#define MODE_MID 1455
#define MODE_MAX 1870

void rcInit(void);
uint8_t checkPulse(uint16_t pw);
float map(float value, float fromLow, float fromHigh, float toLow, float toHigh);
int32_t bound(int32_t val, int32_t lowLimit, int32_t highLimit);
uint8_t getMode(uint16_t ch, uint16_t estop, uint16_t lowPoint, uint16_t midPoint, uint16_t highPoint, uint16_t threshold);

#endif
