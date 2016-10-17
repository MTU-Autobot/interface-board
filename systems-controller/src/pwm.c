/*
 * pwm.c
 * configuration and function for pwm
 */

#include "kinetis.h"
#include "pwm.h"

void pwmInit(void){
    // enable the FTM
    FTM0_MODE = FTM_MODE_WPDIS;
    FTM0_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;

    // set clock to system clock and divide by 32, set MOD to 44999 for 50Hz
    FTM0_MODE |= FTM_MODE_WPDIS;
    FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(5)
    FTM0_MOD = 0xAFC7;

    // edge aligned pwm, clear output on match
    FTM0_C0SC = FTM_CSC_MSB | FTM_CSC_ELSB;
    FTM0_C1SC = FTM_CSC_MSB | FTM_CSC_ELSB;

    // configure output pins
    PORTC_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_DSE;
    PORTC_PCR2 = PORT_PCR_MUX(4) | PORT_PCR_DSE;
}
