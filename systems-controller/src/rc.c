/*
 * rc.c
 * RC controller init and functions
 */

#include "kinetis.h"

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
