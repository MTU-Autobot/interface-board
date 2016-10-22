#include <stdio.h>
#include <string.h>
#include <math.h>
#include "QuadDecode_def.h"

extern "C" {
    #include "kinetis.h"
    #include "serial.h"
    #include "rc.h"
    #include "pwm.h"
}

// speed calculation stuff
// encoders are 512ppr, 20:1 gearbox, 4 counts/count? (not sure why this happens,
// its just what the encoder module produces)
#define CNTS_PER_REV 512 * 20 * 4
#define RPM_TO_SPEED (29.0 * PI * 60.0) / (12.0 * 5280.0)

// stack light pins
#define SL_RED 0x01
#define SL_YELLOW 0x02
#define SL_GREEN 0x04

volatile uint32_t micross = 0;

// variables for rc receiver
volatile uint32_t ch0rise = 0;
volatile uint32_t ch0pw = 0;
volatile uint8_t ch0good = 0;

volatile uint32_t ch1rise = 0;
volatile uint32_t ch1pw = 0;
volatile uint8_t ch1good = 0;

volatile uint32_t ch2rise = 0;
volatile uint32_t ch2pw = 0;
volatile uint8_t ch2good = 0;

volatile uint32_t ch3rise = 0;
volatile uint32_t ch3pw = 0;
volatile uint8_t ch3good = 0;

// encoder variables
int32_t rightEncPos = 0;
int32_t leftEncPos = 0;
volatile int32_t lastRightEncPos = 0;
volatile int32_t lastLeftEncPos = 0;
volatile float rightRPM = 0;
volatile float leftRPM = 0;


// interrupt every 1us
void pit0_isr(void){
    // clear interrupt flag
    PIT_TFLG0 = 0x01;
    micross++;
}

// interrupt every 1ms
void pit1_isr(void){
    //clear flag
    PIT_TFLG1 = 0x01;

    // calculate RPM
    // (change in pos) * 1000 ms/s * 60 s/min / conversion factor
    rightRPM = ((float)(rightEncPos - lastRightEncPos) * 1000 * 60.0) / CNTS_PER_REV;
    leftRPM = ((float)(leftEncPos - lastLeftEncPos) * 1000 * 60.0) / CNTS_PER_REV;

    lastRightEncPos = rightEncPos;
    lastLeftEncPos = leftEncPos;
}

// pin change interrupt for rc controller
void portd_isr(void){
    // determine which pin changed
    if(PORTD_PCR0 & PORT_PCR_ISF){
        // clear the flag
        PORTD_PCR0 |= PORT_PCR_ISF;
        // get pulse width
        if(GPIOD_PDIR & 0x01){
            ch0rise = micross;
        }else{
            ch0pw = micross - ch0rise;
            ch0good = 1;
        }
    }else if(PORTD_PCR1 & PORT_PCR_ISF){
        // if not PD0, then PD1
        PORTD_PCR1 |= PORT_PCR_ISF;
        // get pulse width
        if(GPIOD_PDIR & 0x02){
            ch1rise = micross;
        }else{
            ch1pw = micross - ch1rise;
            ch1good = 1;
        }
    }else if(PORTD_PCR2 & PORT_PCR_ISF){
        PORTD_PCR2 |= PORT_PCR_ISF;
        // get pulse width
        if(GPIOD_PDIR & 0x04){
            ch2rise = micross;
        }else{
            ch2pw = micross - ch2rise;
            ch2good = 1;
        }
    }else if(PORTD_PCR3 & PORT_PCR_ISF){
        PORTD_PCR3 |= PORT_PCR_ISF;
        // get pulse width
        if(GPIOD_PDIR & 0x08){
            ch3rise = micross;
        }else{
            ch3pw = micross - ch3rise;
            ch3good = 1;
        }
    }
}

// create two encoder objects
QuadDecode<1> rightEnc;
QuadDecode<2> leftEnc;

int main(){
    // PORTC_PCR5 0xxx x001 x100 x000
    // Pin Mux Control (10-8)=0x001, Drive Strength Enable (6)=1
    PORTC_PCR5 = PORT_PCR_MUX(1) | PORT_PCR_DSE;
    GPIOC_PDDR |= 0x20;
    GPIOC_PDOR |= 0x20;

    // stack light pin configuration, PORTB 0-2
    PORTB_PCR0 = PORT_PCR_MUX(1) | PORT_PCR_DSE;
    PORTB_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_DSE;
    PORTB_PCR2 = PORT_PCR_MUX(1) | PORT_PCR_DSE;
    GPIOB_PDDR |= (SL_RED | SL_YELLOW | SL_GREEN);

    // enable PIT module and clocking
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0x00;
    // load 35 for 1us period at 36 MHz?
    PIT_LDVAL0 = 0x23;
    // enable interrupt
    PIT_TCTRL0 = 0x03;
    // 1ms period
    PIT_LDVAL1 = 0x8C9F;
    PIT_TCTRL1 = 0x03;

    // enable PIT interrupts
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
    NVIC_ENABLE_IRQ(IRQ_PIT_CH1);

    // start pwm and set both outputs to 1ms
    pwmInit();
    pwmSetPeriod(PWM1, 2250);
    pwmSetPeriod(PWM2, 2250);
    rcInit();

    // setup and start encoder objects
    rightEnc.setup();
    leftEnc.setup();
    rightEnc.start();
    leftEnc.start();
    rightEnc.zeroFTM();
    leftEnc.zeroFTM();

    char printBuf[32] = "";
    uint32_t currTime = 0;
    uint32_t prevTime = 0;

    // motor variables for manual control
    int16_t ch0Mapped = 0;
    int16_t ch1Mapped = 0;
    int16_t rPl = 0;
    int16_t rMl = 0;
    int16_t rightMotor = 0;
    int16_t leftMotor = 0;

    while(1){
        currTime = micross;

        // get encoder positions
        rightEncPos = rightEnc.calcPosn();
        leftEncPos = leftEnc.calcPosn();

        // calculate motor values
        // see http://home.kendra.com/mauser/Joystick.html for an explaination
        ch0Mapped = 3375 - map(ch0pw, CH0_MIN, CH0_MAX, MAX_PERIOD, MIN_PERIOD);
        ch1Mapped = map(ch1pw, CH1_MIN, CH1_MAX, MIN_PERIOD, MAX_PERIOD);
        rPl = (MAX_PERIOD - abs(ch0Mapped)) * (ch1Mapped / MAX_PERIOD) + ch1Mapped;
        rMl = (MAX_PERIOD - ch1Mapped) * (ch0Mapped / MAX_PERIOD) + ch0Mapped;
        rightMotor = bound(rPl + rMl, MIN_PERIOD, MAX_PERIOD);
        leftMotor = bound(rPl - rMl, MIN_PERIOD, MAX_PERIOD);

        if(currTime - prevTime >= 10000){
            prevTime = currTime;

            float rightSpeed = rightRPM * RPM_TO_SPEED;
            int32_t d1 = rightSpeed;
            float f2 = rightSpeed - d1;
            int32_t d2 = trunc(f2 * 10000);

            //sprintf(printBuf, "right speed: %d.%03d mph\n", (int)d1, (int)d2);
            //sprintf(printBuf, "right rpm: %f\n", rightRPM);
            sprintf(printBuf, "left: %d\tright: %d\n", (int)leftMotor, (int)rightMotor);
            serialPrint(printBuf);
        }

        // clear good flag after loop
        ch0good = 0;
        ch1good = 0;
        ch2good = 0;
        ch3good = 0;
    }
}
