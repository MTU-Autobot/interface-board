#include <stdio.h>
#include <string.h>
#include <math.h>
#include "QuadDecode_def.h"
#include "PID_v1.h"

extern "C" {
    #include "kinetis.h"
    #include "serial.h"
    #include "rc.h"
    #include "pwm.h"
}

// speed calculation stuff
// encoders are 512 counts per revolution, 4 pulses per count. 20:1 gearbox
#define WHEEL_DIAMETER 2 * 0.371475
#define CNTS_PER_REV_WHEEL 512 * 4 * 20
#define CNTS_PER_REV 512 * 4
#define RPM_TO_SPEED (WHEEL_DIAMETER * PI) / 60.0
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


// encoder and motor variables
#define PID_KP 2.0
#define PID_KI 1.0
#define PID_KD 0.0

int32_t rightEncPos = 0;
int32_t leftEncPos = 0;
volatile int32_t lastRightEncPos = 0;
volatile int32_t lastLeftEncPos = 0;
volatile int32_t rightEncChange = 0;
volatile int32_t leftEncChange = 0;


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
    rightEncChange = rightEncPos - lastRightEncPos;
    leftEncChange = leftEncPos - lastLeftEncPos;

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


// create two encoder objects
QuadDecode<1> rightEnc;
QuadDecode<2> leftEnc;

// create PID objects for each wheel
double leftPidSetpoint, leftPidInput, leftPidOutput = 0.0;
double rightPidSetpoint, rightPidInput, rightPidOutput = 0.0;
PID leftPID(&leftPidInput, &leftPidOutput, &leftPidSetpoint, PID_KP, PID_KI, PID_KD, REVERSE);
PID rightPID(&rightPidInput, &rightPidOutput, &rightPidSetpoint, PID_KP, PID_KI, PID_KD, REVERSE);

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
    NVIC_ENABLE_IRQ(IRQ_PIT_CH1);

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

    // setup PID
    leftPID.SetMode(AUTOMATIC);
    leftPID.SetOutputLimits(-100, 100);
    rightPID.SetMode(AUTOMATIC);
    rightPID.SetOutputLimits(-100, 100);

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

    while(1){
        // get encoder positions and calculate distances
        rightEncPos = -1 * rightEnc.calcPosn();
        leftEncPos = leftEnc.calcPosn();
        currTime = micross;

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
            rightMotor = map(rightMotor, MIN_PERIOD, MAX_PERIOD, LOW_LIMIT_PERIOD-50, HIGH_LIMIT_PERIOD+50);
            leftMotor = map(leftMotor, MIN_PERIOD, MAX_PERIOD, LOW_LIMIT_PERIOD, HIGH_LIMIT_PERIOD);

            /*
            // PID computation
            leftPidInput = map(leftEncChange, -52, 52, -100, 100);
            rightPidInput = map(rightEncChange, -52, 52, -100, 100);
            leftPidSetpoint = map(leftMotor, MIN_PERIOD, MAX_PERIOD, 100, -100);
            rightPidSetpoint = map(rightMotor, MIN_PERIOD, MAX_PERIOD, 100, -100);
            leftPID.Compute();
            rightPID.Compute();

            //leftMotor = map(leftPidOutput, -100, 100, LOW_LIMIT_PERIOD, HIGH_LIMIT_PERIOD);
            rightMotor = map(rightPidOutput, -100, 100, LOW_LIMIT_PERIOD, HIGH_LIMIT_PERIOD);

            if(currTime - prevTime >= 10000){
                prevTime = currTime;
                sprintf(printBuf, "input: %d\tsetpoint: %d\toutput: %d\n", (int)leftPidInput, (int)leftPidSetpoint, (int)leftMotor);
                //sprintf(printBuf, "left: %d\tright: %d\n", (int)leftEncChange, (int)rightEncChange);
                serialPrint(printBuf);
            }
            */


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

            // serial messages defined in serial.h
            if(serialRead(recvBuf, 128, '\n')){
                if(atoi(recvBuf) == ENCODER_MSG){
                    // create string with message number, encoder positions, and current time
                    sprintf(printBuf, "%d\t%d\t%d\t%u\n", (int)msgNum++, (int)leftEncPos, (int)rightEncPos, (unsigned int)currTime);
                    serialPrint(printBuf);
                }else if(atoi(recvBuf) == DRIVE_MSG){
                    char driveMsg[128] = "";
                    // read drive data and move it to a new string for operating on
                    while(!serialRead(recvBuf, 128, '\n'));
                    strcpy(driveMsg, recvBuf);

                    // split string and store drive commands in array
                    static uint16_t wheelVels[] = {2047, 2047};
                    char * token;
                    token = strtok(driveMsg, "\t");
                    wheelVels[0] = atoi(token);
                    token = strtok(NULL, "\t");
                    wheelVels[1] = atoi(token);

                    // get motor speeds and update outputs
                    leftMotor = map(wheelVels[0], 4095, 0, LOW_LIMIT_PERIOD, HIGH_LIMIT_PERIOD);
                    rightMotor = map(wheelVels[1], 4095, 0, LOW_LIMIT_PERIOD, HIGH_LIMIT_PERIOD);

                    if(!failsafe){
                        pwmSetPeriod(PWM1, rightMotor);
                        pwmSetPeriod(PWM2, leftMotor);
                    }else{
                        pwmSetPeriod(PWM1, MID_PERIOD);
                        pwmSetPeriod(PWM2, MID_PERIOD);
                    }

                    //serialPrint(driveMsg);
                }
            }
        }else{
            // also estop mode so kill everything, hopefully without fire 🔥
            GPIOB_PDOR = SL_RED;

            pwmSetPeriod(PWM1, MID_PERIOD);
            pwmSetPeriod(PWM2, MID_PERIOD);
        }

        /*
        if(currTime - prevTime >= 10000){
            prevTime = currTime;


            float rightSpeed = rightRPM * RPM_TO_SPEED;
            int32_t d1 = rightSpeed;
            float f2 = rightSpeed - d1;
            int32_t d2 = trunc(f2 * 10000);


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
        */
    }
}
