/*
 * pwm.c
 * configuration and function for pwm
 */

#include "kinetis.h"
#include "pwm.h"

void pwmInit(void){
    // configure output pins
    //PORTC_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_DSE;
    //PORTC_PCR2 = PORT_PCR_MUX(4) | PORT_PCR_DSE;

    /*
    // enable the FTM
    FTM0_MODE = FTM_MODE_WPDIS;
    FTM0_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;

    // set clock to system clock and divide by 16, set MOD to 44999 for 50Hz
    // "system clock" actually refers to the bus clock for the FTMs, which is
    // the actual system clock divided by 2. freescale pls
    FTM0_SC = 0;
    FTM0_MOD = PWM_PERIOD;
    FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(4);

    // edge aligned pwm, clear output on match
    FTM0_C0SC = FTM_CSC_MSB | FTM_CSC_ELSB;
    FTM0_C1SC = FTM_CSC_MSB | FTM_CSC_ELSB;
    */
    PORTC_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
    PORTC_PCR2 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;

    FTM0_CNT = 0;
    FTM0_MOD = PWM_PERIOD;
    FTM0_C0SC = 0x28;
    FTM0_C1SC = 0x28;
    //FTM0_C2SC = 0x28;
    //FTM0_C3SC = 0x28;
    //FTM0_C4SC = 0x28;
    //FTM0_C5SC = 0x28;
    //FTM0_C6SC = 0x28;
    //FTM0_C7SC = 0x28;
    FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(4);
    FTM0_MODE = FTM_MODE_WPDIS;
    //FTM0_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
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
            //FTM0_SC = 0;
            FTM0_C0V = period;
            //FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(4);
            break;
        case 1:
            //FTM0_SC = 0;
            FTM0_C1V = period;
            //FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(4);
            break;
        default:
            break;
    }
}
