/**
* 1ms process for PIDF closed-loop.
* @param pid ptr to pid object
* @param pos signed integral position (or velocity when in velocity mode).
* The target pos/velocity is ramped into the target member from caller's 'in'.
* If the CloseLoopRamp in the selected Motor Controller Profile is zero then
* there is no ramping applied. (throttle units per ms)
* PIDF is traditional, unsigned coefficients for P,i,D, signed for F.
* Target pos/velocity is feed forward.
*
* Izone gives the abilty to autoclear the integral sum if error is wound up.
* @param revMotDuringCloseLoopEn nonzero to reverse PID output direction.
* @param oneDirOnly when using positive only sensor, keep the closed-loop from outputing negative throttle.
*/
void PID_Calc1Ms(pid_t * pid, int32_t pos,uint8_t revMotDuringCloseLoopEn, uint8_t oneDirOnly){
    /* grab selected slot */
    MotorControlProfile_t * slot = MotControlProf_GetSlot();
    /* calc error : err = target - pos*/
    int32_t err = pid->target - pos;
    pid->err = err;
    /*abs error */
    int32_t absErr = err;
    if(err < 0)
        absErr = -absErr;

    /* integrate error */
    if(0 == pid->notFirst){
        /* first pass since reset/init */
        pid->iAccum = 0;
        /* also tare the before ramp throt */
        pid->out = BDC_GetThrot(); /* the save the current ramp */
    }else if((!slot->IZone) || (absErr < slot->IZone) ){
        /* izone is not used OR absErr is within iZone */
        pid->iAccum += err;
    }else{
        pid->iAccum = 0;
    }

    /* dErr/dt */
    if(pid->notFirst){
        /* calc dErr */
        pid->dErr = (err - pid->prevErr);
    }else{
        /* clear dErr */
        pid->dErr = 0;
    }

    /* P gain X the distance away from where we want */
    pid->outBeforRmp = PID_Mux_Unsigned(err, slot->P);

    if(pid->iAccum && slot->I){
        /* our accumulated error times I gain. If you want the robot to creep up then pass a nonzero Igain */
        pid->outBeforRmp += PID_Mux_Unsigned(pid->iAccum, slot->I);
    }

    /* derivative gain, if you want to react to sharp changes in error (smooth things out). */
    pid->outBeforRmp += PID_Mux_Unsigned(pid->dErr, slot->D);
    /* feedforward on the set point */
    pid->outBeforRmp += PID_Mux_Signed(pid->target, slot->F);

    /* arm for next pass */
    {
        pid->prevErr = err; /* save the prev error for D */
        pid->notFirst = 1; /* already serviced first pass */
    }

    /* if we are using one-direction sensor, only allow throttle in one dir.
    If it's the wrong direction, use revMotDuringCloseLoopEn to flip it */
    if(oneDirOnly){
        if(pid->outBeforRmp < 0)
            pid->outBeforRmp = 0;
    }

    /* honor the direction flip from control */
    if(revMotDuringCloseLoopEn)
        pid->outBeforRmp = -pid->outBeforRmp;

    /* honor closelooprampratem, ramp out towards outBeforRmp */
    if(0 != slot->CloseLoopRampRate){
        if(pid->outBeforRmp >= pid->out){
            /* we want to increase our throt */
            int32_t deltaUp = pid->outBeforRmp - pid->out;

            if(deltaUp > slot->CloseLoopRampRate)
                deltaUp = slot->CloseLoopRampRate;
            pid->out += deltaUp;
        }else{
            /* we want to decrease our throt */
            int32_t deltaDn = pid->out - pid->outBeforRmp;

            if(deltaDn > slot->CloseLoopRampRate)
                deltaDn = slot->CloseLoopRampRate;
            pid->out -= deltaDn;
        }
    }else{
        pid->out = pid->outBeforRmp;
    }
}
