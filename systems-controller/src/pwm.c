/*
 * pwm.c
 * configuration and function for pwm
 */

#include "kinetis.h"
#include "pwm.h"

void pwmInit(void){
    // configure output pins
    PORTC_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
    PORTC_PCR2 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;

    // set up ftm, FTM_MODE_FTMEN seems to break things
    FTM0_CNT = 0;
    FTM0_MOD = PWM_PERIOD;

    // edge aligned pwm, clear output on match
    FTM0_C0SC = FTM_CSC_MSB | FTM_CSC_ELSB;
    FTM0_C1SC = FTM_CSC_MSB | FTM_CSC_ELSB;

    // system clock (bus clock is real system clock div by 2) div by 16
    FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(4);
}

void pwmSetPeriod(uint8_t pin, uint16_t period){
    // make sure period is in bounds
    if(period > PWM_PERIOD){
        period = PWM_PERIOD;
    }else if(period < 0){
        period = 0;
    }

    // set the correct period register, pin values are associated in pwm.h
    switch(pin){
        case 0:
            FTM0_C0V = period;
            break;
        case 1:
            FTM0_C1V = period;
            break;
        default:
            break;
    }
}
