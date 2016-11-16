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
// encoders are 512 counts per revolution, 4 pulses per count. 20:1 gearbox
#define WHEEL_DIAMETER 29.0
#define CNTS_PER_REV_WHEEL 512 * 4 * 20
#define CNTS_PER_REV 512 * 4
#define RPM_TO_SPEED (29.0 * PI * 60.0) / (12.0 * 5280.0)
#define TICKS_TO_DIST (PI * WHEEL_DIAMETER) / CNTS_PER_REV_WHEEL

// stack light pins and mode
#define SL_RED 0x01
#define SL_YELLOW 0x04
#define SL_GREEN 0x02
#define MODE_ESTOP 0
#define MODE_MANUAL 1
#define MODE_AUTO 2

// timekeeping
#define FS_TIME 500000
#define BLINK_TIME 750000
volatile uint32_t micross = 0;

// variables for rc receiver
#define NUM_CHANNELS 4
#define RC_X 0
#define RC_Y 1
#define RC_ESTOP 2
#define RC_MODE 3

volatile uint32_t chRise[NUM_CHANNELS];
volatile uint16_t ch[NUM_CHANNELS];
volatile uint8_t pwGood[NUM_CHANNELS];

/*
// encoder and motor variables
#define KP 0.0
#define KI 0.0
#define KD 0.0

int32_t rightEncPos = 0;
int32_t leftEncPos = 0;
volatile int32_t lastRightEncPos = 0;
volatile int32_t lastLeftEncPos = 0;
volatile float rightRPM = 0;
volatile float leftRPM = 0;
*/


// interrupt every 1us
void pit0_isr(void){
    // clear interrupt flag
    PIT_TFLG0 = 0x01;
    micross++;
}

/*
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
*/

// pin change interrupt for rc controller
void portd_isr(void){
    // determine which pin changed
    if(PORTD_PCR0 & PORT_PCR_ISF){
        // clear the flag
        PORTD_PCR0 |= PORT_PCR_ISF;
        // get pulse width
        if(GPIOD_PDIR & 0x01){
            chRise[RC_X] = micross;
        }else{
            ch[RC_X] = micross - chRise[RC_X];
            pwGood[RC_X] = 1;
        }
    }else if(PORTD_PCR1 & PORT_PCR_ISF){
        // if not PD0, then PD1
        PORTD_PCR1 |= PORT_PCR_ISF;
        // get pulse width
        if(GPIOD_PDIR & 0x02){
            chRise[RC_Y] = micross;
        }else{
            ch[RC_Y] = micross - chRise[RC_Y];
            pwGood[RC_Y] = 1;
        }
    }else if(PORTD_PCR2 & PORT_PCR_ISF){
        PORTD_PCR2 |= PORT_PCR_ISF;
        // get pulse width
        if(GPIOD_PDIR & 0x04){
            chRise[RC_ESTOP] = micross;
        }else{
            ch[RC_ESTOP] = micross - chRise[RC_ESTOP];
            pwGood[RC_ESTOP] = 1;
            //serialPrint("wtf\n");
        }
    }else if(PORTD_PCR3 & PORT_PCR_ISF){
        PORTD_PCR3 |= PORT_PCR_ISF;
        // get pulse width
        if(GPIOD_PDIR & 0x08){
            chRise[RC_MODE] = micross;
        }else{
            ch[RC_MODE] = micross - chRise[RC_MODE];
            pwGood[RC_MODE] = 1;
        }
    }
}

// function to calculate distance moved
float * calcDistance(int32_t rightVal, int32_t leftVal){
    static float distArray[2] = {0};
    static int32_t lastRightVal = 0;
    static int32_t lastLeftVal = 0;

    // calculate time since last run
    static uint32_t then = micross;
    uint32_t now = micross;
    uint32_t dTime = now - then;
    then = now;

    // calculate distance
    int32_t rightDist = (rightVal - lastRightVal) * TICKS_TO_DIST;
    int32_t leftDist = (leftVal - lastLeftVal) * TICKS_TO_DIST;
    distArray[0] = rightDist / (float)(dTime / 1000000);
    distArray[1] = leftDist / (float)(dTime / 1000000);
    char test[32];
    sprintf(test, "rightDist: %d\tleftDist: %d\n", (int)distArray[0], (int)distArray[1]);
    serialPrint(test);

    // update last position
    lastRightVal = rightVal;
    lastLeftVal = leftVal;

    return distArray;
}

// convert a float ot a printable string because dumb reasons
void float2str(char * str, float fl){
    int32_t d1 = fl;                // Get the integer part (678).
    float f2 = fl - d1;             // Get fractional part (0.01234567).
    int32_t d2 = trunc(f2 * 10000); // Turn into integer (123).
    float f3 = f2 * 10000 - d2;     // Get next fractional part (0.4567).
    int32_t d3 = trunc(f3 * 10000); // Turn into integer (4567).
    sprintf(str, "%d.%04d%04d\n", (int)d1, (int)d2, (int)d3);
}

// create two encoder objects
QuadDecode<1> rightEnc;
QuadDecode<2> leftEnc;

int main(){
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
    //NVIC_ENABLE_IRQ(IRQ_PIT_CH1);

    // start pwm and set both outputs to 1.5ms
    pwmInit();
    pwmSetPeriod(PWM1, MID_PERIOD);
    pwmSetPeriod(PWM2, MID_PERIOD);
    rcInit();

    // setup and start encoder objects
    rightEnc.setup();
    leftEnc.setup();
    rightEnc.start();
    leftEnc.start();
    rightEnc.zeroFTM();
    leftEnc.zeroFTM();

    char printBuf[128] = "";
    char recvBuf[128] = "";

    // timekeeping
    uint32_t currTime = 0;
    uint32_t prevTime = 0;
    uint32_t endTime = 0;
    uint32_t fsTimer = 0;
    uint32_t blinkTimer = 0;

    // misc mode and channel variables
    uint8_t failsafe = 1;
    uint16_t chLast[NUM_CHANNELS];
    uint8_t mode = MODE_ESTOP;

    // motor variables for manual control
    int16_t ch0Mapped = 0;
    int16_t ch1Mapped = 0;
    int16_t rPl = 0;
    int16_t rMl = 0;
    int16_t rightMotor = 0;
    int16_t leftMotor = 0;
    float * distVals;

    while(1){
        //currTime = micross;

        // check pulse widths
        for(uint8_t i = 0; i < NUM_CHANNELS; i++){
            if(checkPulse(ch[i])){
                chLast[i] = ch[i];
            }else{
                pwGood[i] = 0;
                ch[i] = chLast[i];
            }
        }

        //failsafe check
        if(micross - fsTimer > FS_TIME){
            fsTimer = micross;

            for(uint8_t i = 0; i < NUM_CHANNELS; i++){
                if(pwGood[i]){
                    failsafe = 0;
                }else{
                    failsafe = 1;
                    mode = MODE_ESTOP;
                    break;
                }
            }

            // clear good flag after check
            for(uint8_t i = 0; i < NUM_CHANNELS; i++){
                pwGood[i] = 0;
            }
        }

        // get operation mode
        if(!failsafe){
            mode = getMode(ch[RC_MODE], ch[RC_ESTOP], MODE_MIN, MODE_MID, MODE_MAX, RC_THRESH);
        }

        if(mode == MODE_ESTOP){
            // stack light to red
            GPIOB_PDOR = SL_RED;

            // kill motors
            pwmSetPeriod(PWM1, MID_PERIOD);
            pwmSetPeriod(PWM2, MID_PERIOD);
        }else if(mode == MODE_MANUAL){
            // stack light to yellow
            GPIOB_PDOR = SL_YELLOW;

            // calculate motor values, single stick to tank drive
            // see http://home.kendra.com/mauser/Joystick.html for an explaination
            ch0Mapped = MID_PERIOD - (int16_t)map(ch[RC_X], CH0_MIN, CH0_MAX, MIN_PERIOD, MAX_PERIOD);
            ch1Mapped = (int16_t)map(ch[RC_Y], CH1_MIN, CH1_MAX, MAX_PERIOD, MIN_PERIOD);
            rPl = (MAX_PERIOD - abs(ch0Mapped)) * (ch1Mapped / MAX_PERIOD) + ch1Mapped;
            rMl = (MAX_PERIOD - ch1Mapped) * (ch0Mapped / MAX_PERIOD) + ch0Mapped;
            rightMotor = bound(rPl + rMl, MIN_PERIOD, MAX_PERIOD);
            leftMotor = bound(rPl - rMl, MIN_PERIOD, MAX_PERIOD);

            // apply speed limits
            rightMotor = map(rightMotor, MIN_PERIOD, MAX_PERIOD, LOW_LIMIT_PERIOD, HIGH_LIMIT_PERIOD);
            leftMotor = map(leftMotor, MIN_PERIOD, MAX_PERIOD, LOW_LIMIT_PERIOD, HIGH_LIMIT_PERIOD);

            // update outputs
            if(!failsafe){
                pwmSetPeriod(PWM1, rightMotor);
                pwmSetPeriod(PWM2, leftMotor);
            }else{
                pwmSetPeriod(PWM1, MID_PERIOD);
                pwmSetPeriod(PWM2, MID_PERIOD);
            }
        }else if(mode == MODE_AUTO){
            static uint32_t msgNum = 0;

            // blink green for auto mode
            GPIOB_PCOR = SL_RED | SL_YELLOW;
            if(micross - blinkTimer > BLINK_TIME){
                blinkTimer = micross;
                // toggle output
                GPIOB_PTOR = SL_GREEN;
            }

            if(serialRead(recvBuf, 128, '\n')){
                //strcat(recvBuf, "\n");
                //serialPrint(recvBuf);

                if(atoi(recvBuf) == ENCODER_MSG){
                    // get encoder position
                    int32_t rightEncPos = -1 * rightEnc.calcPosn();
                    int32_t leftEncPos = leftEnc.calcPosn();
                    currTime = micross;

                    // send data
                    sprintf(printBuf, "%d\t%d\t%d\t%u\n", (int)msgNum++, (int)leftEncPos, (int)rightEncPos, (unsigned int)currTime);
                    serialPrint(printBuf);
                }
            }
        }else{
            // also estop mode so kill everything, hopefully without fire 🔥
            GPIOB_PDOR = SL_RED;

            pwmSetPeriod(PWM1, MID_PERIOD);
            pwmSetPeriod(PWM2, MID_PERIOD);
        }

        if(currTime - prevTime >= 10000){
            prevTime = currTime;

            /*
            float rightSpeed = rightRPM * RPM_TO_SPEED;
            int32_t d1 = rightSpeed;
            float f2 = rightSpeed - d1;
            int32_t d2 = trunc(f2 * 10000);
            */

            //sprintf(printBuf, "right speed: %d.%03d mph\n", (int)d1, (int)d2);
            //sprintf(printBuf, "right rpm: %f\n", rightRPM);
            //sprintf(printBuf, "rc_x: %d\nrc_y: %d\nrc_estop: %d\nrc_mode: %d\n\n", (int)pwGood[RC_X], (int)pwGood[RC_Y], (int)pwGood[RC_ESTOP], (int)pwGood[RC_MODE]);
            //sprintf(printBuf, "rc_x: %d\nrc_y: %d\nrc_estop: %d\nrc_mode: %d\n\n", (int)ch[RC_X], (int)ch[RC_Y], (int)ch[RC_ESTOP], (int)ch[RC_MODE]);
            //sprintf(printBuf, "mode: %d failsafe: %d count: %d\n", (int)mode, (int)failsafe, i++);
            //sprintf(printBuf, "left encoder: %d right encoder: %d time: %d\r\n", (int)leftEncPos, (int)rightEncPos, currTime);
            //sprintf(printBuf, "%d\t%d\t%d\t%u\r\n", i++, (int)leftEncPos, (int)rightEncPos, (unsigned int)currTime);
            //serialPrint(printBuf);
        }

        endTime = micross;
    }
}

/*
float rightPID(float cmd, float target, float actual){
    float error = 0.0;
    float pidTerm = 0.0;
    float derivative = 0.0;
    static float integral = 0.0;
    static float lastError = 0.0;

    error = abs(target) - abs(actual);
    integral += error;
    derivative = error - lastError;
    pidTerm = (KP * error) + (KI * integral) + (KD * derivative);
    return cmd + pidTerm;
}

float leftPID(float cmd, float target, float actual){
    float error = 0.0;
    float pidTerm = 0.0;
    float derivative = 0.0;
    static float integral = 0.0;
    static float lastError = 0.0;

    error = abs(target) - abs(actual);
    integral += error;
    derivative = error - lastError;
    pidTerm = (KP * error) + (KI * integral) + (KD * derivative);
    return cmd + pidTerm;
}
*/
