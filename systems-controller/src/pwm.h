/*
 * pwm.h
 * function prototypes for pwm
 */

#ifndef PWM_H_
#define PWM_H_

#define PWM_PERIOD 0xAFC7
#define PWM1 0
#define PWM2 1
#define MIN_PERIOD 2250
#define MAX_PERIOD 4500

void pwmInit(void);
void pwmSetPeriod(uint8_t pin, uint16_t period);

#endif
