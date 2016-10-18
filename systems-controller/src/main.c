#include <stdio.h>
#include <string.h>
#include "kinetis.h"
#include "serial.h"
#include "rc.h"
#include "pwm.h"

volatile uint32_t micross = 0;

volatile uint32_t ch1rise = 0;
volatile uint32_t ch1pw = 0;
volatile uint32_t ch2rise = 0;
volatile uint32_t ch2pw = 0;
volatile uint32_t ch3rise = 0;
volatile uint32_t ch3pw = 0;
volatile uint32_t ch4rise = 0;
volatile uint32_t ch4pw = 0;

void pit0_isr(void){
    // clear interrupt flag
    PIT_TFLG0 = 0x01;
    micross++;
}

void ftm1_isr(void){
    //
}

// pin change interrupt for rc controller
void portd_isr(void){
    // determine which pin changed
    if(PORTD_PCR0 & PORT_PCR_ISF){
        // clear the flag
        PORTD_PCR0 |= PORT_PCR_ISF;
        // get pulse width
        if(GPIOD_PDIR & 0x01){
            ch1rise = micross;
        }else{
            ch1pw = micross - ch1rise;
        }
    }else if(PORTD_PCR1 & PORT_PCR_ISF){
        // if not PD0, then PD1
        PORTD_PCR1 |= PORT_PCR_ISF;
        // get pulse width
        if(GPIOD_PDIR & 0x02){
            ch2rise = micross;
        }else{
            ch2pw = micross - ch2rise;
        }
    }else if(PORTD_PCR2 & PORT_PCR_ISF){
        PORTD_PCR2 |= PORT_PCR_ISF;
        // get pulse width
        if(GPIOD_PDIR & 0x04){
            ch3rise = micross;
        }else{
            ch3pw = micross - ch3rise;
        }
    }else if(PORTD_PCR3 & PORT_PCR_ISF){
        PORTD_PCR3 |= PORT_PCR_ISF;
        // get pulse width
        if(GPIOD_PDIR & 0x08){
            ch4rise = micross;
        }else{
            ch4pw = micross - ch4rise;
        }
    }
}


int main(){
    // PORTC_PCR5 0xxx x001 x100 x000
    // Pin Mux Control (10-8)=0x001, Drive Strength Enable (6)=1
    PORTC_PCR5 = PORT_PCR_MUX(1) | PORT_PCR_DSE;
    GPIOC_PDDR |= 0x20;
    GPIOC_PDOR |= 0x20;

    // encoder pin setup
    PORTA_PCR12 = PORT_PCR_MUX(7) | PORT_PCR_PFE | PORT_PCR_PE;
    PORTA_PCR13 = PORT_PCR_MUX(7) | PORT_PCR_PFE | PORT_PCR_PE;

    // enable PIT module and clocking
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0x00;
    // load 35 for 1us period at 36 MHz?
    PIT_LDVAL0 = 0x23;
    // enable interrupt
    PIT_TCTRL0 = 0x03;

    // FTM setup
    FTM1_MODE = FTM_MODE_WPDIS;
    FTM1_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
    // zero out registers just in case
    FTM1_CNT = 0;
    FTM1_MOD = 0;
    FTM1_C0SC = 0;
    FTM1_C1SC = 0;
    FTM1_SC = 0;
    // set registers for filter
    FTM1_FILTER = FTM_FILTER_CH1FVAL(2) | FTM_FILTER_CH0FVAL(2);
    FTM1_CNTIN = 0;
    FTM1_MOD = 0xFFFF;
    FTM1_CNT = 0;
    // set registers for output compare mode
    FTM1_COMBINE = 0;
    FTM1_C0SC = FTM_CSC_MSA;
    FTM1_C0V = 0x6000;
    // quadrature control
    FTM1_QDCTRL = FTM_QDCTRL_PHAPOL | FTM_QDCTRL_PHBPOL | FTM_QDCTRL_QUADEN;
    FTM1_FMS = FTM_FMS_WPEN;

    // enable PIT and PORT and FTM interrupts
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
    NVIC_ENABLE_IRQ(IRQ_PORTD);
    NVIC_ENABLE_IRQ(IRQ_FTM1);

    pwmInit();
    pwmSetPeriod(PWM1, 2250);
    pwmSetPeriod(PWM2, 4500);

    while(1){
    }
}
