#include "Interpolation.h"

double interpolateTrap(double error, double velocity){
    static const double LEAP_PERCENTAGE = .5;
    static const double MIN_STEP = .0001;

    double output = error;

    if((fabs(error) > MIN_STEP)){
        output = (LEAP_PERCENTAGE * error);

        if(fabs(output) > velocity)
            output = output < 0 ? -velocity : velocity;
    }
    return output;
}


/*
 * theta(t) = vt^4 + wt^3 + xt^2 + yt + z
 * z = ths
 * y = vs
 * x = 1/tf^2 * ( ( 5 ( ths - thf + tf*vs ) - 16 ( ths - thv + tf*vs ) - tf*vs ) )
 * w = 1/tf^3 * ( ( 32 ( ths - thv + tf*vs/2 ) - 14 ( ths - thf + tf*vs) + 3*tf*vs ) )
 * v = 2/tf^4 * ( ( 4 ( ths - thf + tfvs ) - 8 ( ths - thv + tf*vs/2 ) - 2*tf*vs ) )
*/
FourthOrderParams initFourthOrder(double ths, double vs, double thv, double thf, double tf ){
    struct FourthOrderParams params;
    memset(&params, 0, sizeof(params));
    if (tf == 0)
        return params;
    double tfvs = tf * vs;
    double tfpow = tf * tf;

    double tf2 = tf * tf;
    double tf3 = tf2 * tf;
    double tf4 = tf3 * tf;

    params.a0 = ths;
    params.a1 = vs;
    //params.a2 =  ( 5 * (ths - thf + tfvs) ) / tf2 - ( 16 * (ths - thv + tfvs/2)) / tf2 - vs / tf;

    params.a2 = ( (5 * (ths - thf + tfvs))  -  (16 * (ths - thv + tfvs/2))  - tfvs ) / tf2;

    //params.a3 = ( 32 * (ths - thv + tfvs/2)) / tf3 - ( 14 * (ths - thf + tfvs)) / tf3 + (3*vs) / tf2;

    params.a3 =  ( ( 32 * (ths - thv + tfvs/2)) - ( 14 * (ths - thf + tfvs)) + (3*tfvs) ) / tf3;

    //params.a4 = ( 8 * (ths - thf + tfvs)) / tf4 - ( 16 * (ths - thv + tfvs/2)) / tf4 - (2*vs) / tf3;

    params.a4 = 2 * ( ( 4 * (ths - thf + tfvs)) - ( 8 * (ths - thv + tfvs/2)) - tfvs ) / tf4;

    params.ths = ths;
    params.tv = tf/2;
    params.tf = tf;
    params.thf = thf;
    params.valid = true;
    return params;
}

FourthOrderParams initFourthOrder(double ths, double vs, double thv, double tv, double thf, double tf){
    struct FourthOrderParams params;
    memset(&params, 0, sizeof(params));
    if (tv == 0 || tf == 0)
        return params;
    double tfvs = tf * vs;
    double tftv = tf * tv;
    double tvvs = tv * vs;
    double tfpow = tf * tf;

    double tf2 = tf * tf;
    double tf3 = tf2 * tf;
    double tf4 = tf3 * tf;

    double tv2 = tv * tv;
    double tv3 = tv2 * tv;
    double tv4 = tv3 * tv;

    params.a0 = ths;
    params.a1 = vs;

    params.a2 =     ((- 3 * tv2 + 4 * tftv) * (ths - thf + tfvs))       / (tf2 * (tf2 - 2 * tftv + tv2))
                  - (tf2 * (ths - thv + tvvs))                          / (tv2 * (tf2 - 2 * tftv + tv2))
                  - (tvvs)                                              / (tf  * (tf - tv));


    params.a3 =     (2 * tf * (ths - thv + tvvs))                       / (tv2 * (tf2 - 2 * tftv + tv2))
                  - (vs * (tf + tv))                                    / (tf  * (-tf2 + tftv))
                  - (2 * (2 * tf2 - tv2) * (ths - thf + tfvs))          / (tf2 * (tf3 - 2 * tf2 * tv + tf * tv2));


    params.a4 =      vs                                                 / (tf  * (-tf2 + tftv) )
                   - (ths - thv + tvvs)                                 / (tv2 * (tf2 - 2 * tftv + tv2) )
                   + ((3 * tf - 2 * tv) * (ths - thf + tfvs))           / (tf2 * (tf3 - 2 * tf2 * tv + tf * tv2) );

    params.ths = ths;
    params.tv = tv;
    params.tf = tf;
    params.thf = thf;
    params.valid = true;
    return params;
}

double totalTime(double start, double end, double vs, double interVel){
    double time = 0;
    double distance = 0;

    const double ACC = 1 * vs < 0 ? -1 : 1;

    const double vf = 0;

    time = fabs((vf - vs)/ACC);
    distance = start + ((vs * time) + (.5 * ACC * time * time));

    return time + fabs((end - distance) / interVel);
    /*
    if ( (vs > 0 && end - start < 0) || (vs < 0 && end - start > 0) ){
        const double ACC = 1;
        const double vf = 0;

        time += fabs((vf - vs)/ACC);
        distance = (vs * time) + (.5 * ACC * time * time);

        time += fabs((end - distance) / interVel);
        return time;
    }
    return fabs((end - start)/interVel);

    return 0;*/
}

double interpolateFourthOrder(FourthOrderParams params, double time){
    if (!params.valid)
        return 0;
    if (time > params.tf || time < 0){
        params.valid = false;
        return 0;
    }
    double t = time;
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;

    return t4*params.a4 + t3*params.a3 + t2*params.a2 + t*params.a1;
}

double interpolateSin(double error, double time){
    return ( error * .5 * (1.0 - cos( M_PI * time ) ) );
}

double interpolateSin1(double error, double time){
    return ( error * (1.0 - cos( .5 * M_PI * time ) ) );
}

double interpolateSin2(double error, double time){
    return ( error * ( -cos( .5 * M_PI * ( time + 1 ) ) ) );
}

double interpolateSin3(double error, double time){
    return ( error * ( sin( .5 * M_PI * time ) ) );
}

double interpolateSin4(double error, double time){
    return ( error * ( time - ( sin( M_PI * time ) / M_PI ) ) );
}

double interpolateSin5(double error, double time){
    return ( error * ( time + ( sin( M_PI * time ) / M_PI ) ) );
}

//Currently unimplemented. Depends on unknown variable JW_temp[6]
double interpolateSin6(double error, double time){
    return 0;
}

double interpolateSin7(double error, double currStepCount, double totalStepCount, double delay){
    double tempMixTime = (currStepCount - delay) / (totalStepCount - delay);
    double time = currStepCount / totalStepCount;

    if(tempMixTime < 0)
        tempMixTime = 0;

    return interpolateSin4(error * 2, tempMixTime) - interpolateSin3(error, time);
    //return ( error * 2.0f * (tempMixTime - sin(M_PI * tempMixTime) / M_PI ) ) - ( error * ( sin( 0.5 * M_PI * time ) ) ) );
}

//Currently unimplemented. Depends on unknown variable JW_temp[6]
double interpolateSin8(double error, double time){
    return 0;
}

//Currently unimplemented. Depends on unknown variable JW_temp[6]
double interpolateSin9(double error, double time){
    return 0;
}

//Currently unimplemented. Depends on unknown variable JW_temp[6]
double interpolateSin10(double error, double time){
    return 0;
}

// Currently unimplemented. Depends on unknown variables UserData[0], UserData[1]
double interpolateSin11(double error, double currStepCount, double totalStepCount, double delay){
    /*
    tempSidePeriod = (float)_walkingInfo[i][j].GoalTimeCount / 2.0f - _walkingInfo[i][j].UserData[1] / INT_TIME;
    tempSideTime = (float)_walkingInfo[i][j].CurrentTimeCount / tempSidePeriod;
    tempSideDelay = 2.0f*_walkingInfo[i][j].UserData[1] / INT_TIME/tempSidePeriod;

    if(tempSideTime < 1.0f)
        tempSideWalk = _walkingInfo[i][j].UserData[0] * 0.5f * (1.0f - cosf(PI * tempSideTime));
    else if( tempSideTime < (1.0f + tempSideDelay) )
        tempSideWalk = _walkingInfo[i][j].UserData[0];
    else if( tempSideTime < (2.0f + tempSideDelay) )
        tempSideWalk = _walkingInfo[i][j].UserData[0] * 0.5f * (1.0f + cosf(PI * (tempSideTime - 1.0f - tempSideDelay)));
    else
        tempSideWalk = 0.0f;

    _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial + _walkingInfo[i][j].RefPatternDelta * 0.5f * (1.0f - cosf(PI * tempTime)) + tempSideWalk;
    */
    return 0;
}

// Currently unimplemented. Depends on unknown variables UserData[0], UserData[1]
double interpolateSin12(double error, double time){
    /*
    tempSidePeriod = (float)_walkingInfo[i][j].GoalTimeCount/2.0f - _walkingInfo[i][j].UserData[1]/INT_TIME;
    tempSideTime = (float)_walkingInfo[i][j].CurrentTimeCount/tempSidePeriod;
    tempSideDelay = 2.0f*_walkingInfo[i][j].UserData[1]/INT_TIME/tempSidePeriod;

    if(tempSideTime < 1.0f)
        tempSideWalk = _walkingInfo[i][j].UserData[0]*0.5f*(1.0f-cosf(PI*tempSideTime));
    else if( tempSideTime < (1.0f+tempSideDelay) )
        tempSideWalk = _walkingInfo[i][j].UserData[0];
    else if( tempSideTime < (2.0f+tempSideDelay) )
        tempSideWalk = _walkingInfo[i][j].UserData[0]*0.5f*(1.0f+cosf(PI*(tempSideTime-1.0f-tempSideDelay)));
    else
        tempSideWalk = 0.0f;

    _walkingInfo[i][j].RefPatternCurrent = _walkingInfo[i][j].RefPatternInitial+_walkingInfo[i][j].RefPatternDelta*0.5f*(1.0f-cosf(PI*tempTime)) - tempSideWalk;
    */
    return 0;
}
