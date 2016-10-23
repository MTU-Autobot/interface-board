/*
 * rc.c
 * RC controller init and functions
 */

#include "kinetis.h"
#include "rc.h"

void rcInit(void){
    // enable interrupt on both edges, gpio
    PORTD_PCR0 = PORT_PCR_IRQC(11) | PORT_PCR_MUX(1);
    PORTD_PCR1 = PORT_PCR_IRQC(11) | PORT_PCR_MUX(1);
    PORTD_PCR2 = PORT_PCR_IRQC(11) | PORT_PCR_MUX(1);
    PORTD_PCR3 = PORT_PCR_IRQC(11) | PORT_PCR_MUX(1);
    PORTD_PCR4 = PORT_PCR_IRQC(11) | PORT_PCR_MUX(1);
    PORTD_PCR5 = PORT_PCR_IRQC(11) | PORT_PCR_MUX(1);
    PORTD_PCR6 = PORT_PCR_IRQC(11) | PORT_PCR_MUX(1);
    PORTD_PCR7 = PORT_PCR_IRQC(11) | PORT_PCR_MUX(1);

    // enable interrupt for portd
    NVIC_ENABLE_IRQ(IRQ_PORTD);
}

// function to check pulse width is within absolute limits
uint8_t checkPulse(uint16_t pw){
    return (pw <= RC_HIGH_LIM && pw >= RC_LOW_LIM);
}

// taken from arduino
int32_t map(int32_t x, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax){
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// quick function to make things bounded
int32_t bound(int32_t val, int32_t lowLimit, int32_t highLimit){
    if(val > highLimit) val = highLimit;
    if(val < lowLimit) val = lowLimit;
    return val;
}

// idk a good description, it sets a value based on switch position
uint8_t getMode(uint16_t ch, uint16_t estop, uint16_t lowPoint, uint16_t midPoint, uint16_t highPoint, uint16_t threshold){
    uint16_t lowBound = ch - threshold;
    uint16_t highBound = ch + threshold;

    uint16_t estopLowBound = estop - threshold;
    uint16_t estopHighBound = estop + threshold;

    if(ESTOP_MIN < estopHighBound && ESTOP_MIN > estopLowBound) return 0;
    if(lowPoint < highBound && lowPoint > lowBound) return 2;
    if(midPoint < highBound && midPoint > lowBound) return 1;
    if(highPoint < highBound && highPoint > lowBound) return 1;
}
