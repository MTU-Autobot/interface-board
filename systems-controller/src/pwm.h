/*
 * pwm.h
 * function prototypes for pwm
 */

#ifndef PWM_H_
#define PWM_H_

#define SPEED_LIMIT 0.25
#define PWM_PERIOD 0xAFC7
#define PWM1 0
#define PWM2 1
#define MIN_PERIOD 2250
#define MAX_PERIOD 4500
#define MID_PERIOD (MIN_PERIOD + MAX_PERIOD) / 2
#define LOW_LIMIT_PERIOD MID_PERIOD - (MID_PERIOD * SPEED_LIMIT)
#define HIGH_LIMIT_PERIOD MID_PERIOD + (MID_PERIOD * SPEED_LIMIT)

void pwmInit(void);
void pwmSetPeriod(uint8_t pin, uint16_t period);

#endif
