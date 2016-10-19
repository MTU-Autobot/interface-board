#include <stdio.h>
#include <string.h>
#include "QuadDecode_def.h"

extern "C" {
    #include "kinetis.h"
    #include "serial.h"
    #include "rc.h"
    #include "pwm.h"
}

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

// create two encoder objects
QuadDecode<1> enc1;
QuadDecode<2> enc2;

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

    // enable PIT module and clocking
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0x00;
    // load 35 for 1us period at 36 MHz?
    PIT_LDVAL0 = 0x23;
    // enable interrupt
    PIT_TCTRL0 = 0x03;

    // enable PIT and PORT and FTM interrupts
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);

    // start pwm and set both outputs to 1ms
    pwmInit();
    pwmSetPeriod(PWM1, 2250);
    pwmSetPeriod(PWM2, 2250);

    // setup and start encoder objects
    enc1.setup();
    enc2.setup();
    enc1.start();
    enc2.start();
    enc1.zeroFTM();
    enc2.zeroFTM();

    while(1){
        char printBuf[32] = "";
        //int32_t enc1Pos = enc1.calcPosn();
        int32_t enc2Pos = enc2.calcPosn();

        if(micross % 5000 <= 10){
            sprintf(printBuf, "enc1Pos: %d\nenc2Pos: %d\n\n", (int)enc2Pos, (int)enc2Pos);
            serialPrint(printBuf); //27382 - -14096
        }
    }
}
