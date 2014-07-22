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
