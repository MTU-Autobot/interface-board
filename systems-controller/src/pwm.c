/*
 * pwm.c
 * configuration and function for pwm
 */

#include "kinetis.h"
#include "pwm.h"

void pwmInit(void){
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
    // configure output pins
    PORTC_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_DSE;
    PORTC_PCR2 = PORT_PCR_MUX(4) | PORT_PCR_DSE;

    SIM_SCGC6 |= SIM_SCGC6_FTM0;
    SIM_SCGC5 |= SIM_SCGC5_PORTC;
    FTM0_CONF = FTM_CONF_BDMMODE(3);
    FTM0_FMS = 0x00;
    FTM0_MODE |= FTM_MODE_WPDIS | FTM_MODE_FTMEN;
    FTM0_MOD = PWM_PERIOD;
    FTM0_C0SC = FTM_CSC_MSB | FTM_CSC_ELSB;
    FTM0_C1SC = FTM_CSC_MSB | FTM_CSC_ELSB;
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
