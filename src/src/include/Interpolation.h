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

#ifndef _INTERPOLATION_H
#define _INTERPOLATION_H

#include <math.h>
#include <string.h>
#include <stdlib.h>

double interpolateTrap(double error, double velocity);

struct FourthOrderParams {
    double a0, a1, a2, a3, a4;
    double ths, tv, tf, thf;
    bool valid;
};

FourthOrderParams initFourthOrder(double ths, double vs, double thv, double thf, double tf );

FourthOrderParams initFourthOrder(double ths, double vs, double thv, double tv, double thf, double tf);

double totalTime(double start, double end, double vel, double interVel);

double interpolateFourthOrder(FourthOrderParams params, double time);

double interpolateSin(double error, double time);

double interpolateSin1(double error, double time);

double interpolateSin2(double error, double time);

double interpolateSin3(double error, double time);

double interpolateSin4(double error, double time);

double interpolateSin5(double error, double time);

double interpolateSin6(double error, double time);

double interpolateSin7(double error, double currStepCount, double totalStepCount, double delay);

double interpolateSin8(double error, double time);

double interpolateSin9(double error, double time);

double interpolateSin10(double error, double time);

double interpolateSin11(double error, double currStepCount, double totalStepCount, double delay);

double interpolateSin12(double error, double time);

#endif
