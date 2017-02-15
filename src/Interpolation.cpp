/*
Copyright (c) 2013, Drexel University, iSchool, Applied Informatics Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "Interpolation.h"

/**
 * Perform trapazoidal interpolation at a certain velocity with some error
 * @param  error    The error term
 * @param  velocity The velocity
 * @return          The trapazoidal interpolation step
 */
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
/**
 * Initialize the Fourth order polynomial interpolation function parameters 
 * @param  ths Theta start, the starting position
 * @param  vs  Velocity start, the starting velocity
 * @param  thv Theta via, the position of the via
 * @param  thf Theta final, the position at the end
 * @param  tf  Time final, The time it will take
 * @return     The parameters for the fourth order function
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

/**
 * Initialize the Fourth order polynomial interpolation function parameters 
 * @param  ths Theta start, the starting position
 * @param  vs  Velocity start, the starting velocity
 * @param  thv Theta via, the position of the via
 * @param  tv  Time via, the time to get to the via
 * @param  thf Theta final, the position at the end
 * @param  tf  Time final, The time it will take
 * @return     The parameters for the fourth order function
 */
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

/**
 * The total time that it will take
 * @param  start    Start point
 * @param  end      End point
 * @param  vs       staring velocity
 * @param  interVel interpolation velocity
 * @return          Returns the time it will take
 */
double totalTime(double start, double end, double vs, double interVel){
    double time = 0;
    double distance = 0;

    const double ACC = 1 * vs < 0 ? -1 : 1;

    const double vf = 0;

    time = fabs((vf - vs)/ACC);
    distance = start + ((vs * time) + (.5 * ACC * time * time));

    return time + fabs((end - distance) / interVel);
}

/**
 * Performs one step of the fourth order interpolation
 * @param  params The parameters for the fourth order interpolation 
 * @param  time   The time step
 * @return        The value of the interpolation step
 */
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

/**
 * Perform a step of sinusoidal interpolation  
 * @param  error Error of the interpolation 
 * @param  time  The time step
 * @return       The value of the interpolation step 
 */
double interpolateSin(double error, double time){
    return ( error * .5 * (1.0 - cos( M_PI * time ) ) );
}

/**
 * Perform a step of sinusoidal interpolation  
 * @param  error Error of the interpolation 
 * @param  time  The time step
 * @return       The value of the interpolation step 
 */
double interpolateSin1(double error, double time){   //used for walking gait creation
    return ( error * (1.0 - cos( .5 * M_PI * time ) ) );
}
