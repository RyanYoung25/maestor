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

/*
 * Scheduler.h
 *
 *  Created on: Dec 17, 2013
 *      Author: solisknight
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#ifndef CLOCK_REALTIME
// Hack so that Eclipse can stop yelling at me because it can't find the stupid symbol.
#   define CLOCK_REALTIME       0
#endif
#ifndef CLOCK_MONOTONIC
#   define CLOCK_MONOTONIC      1
#endif
#ifndef TIMER_ABSTIME
#   define TIMER_ABSTIME        1
#endif

#define NSEC_PER_SECOND 1000000000

#define FREQ_1HZ    1000000000; // (1.00 sec)
#define FREQ_2HZ    500000000;  // (0.50 sec)
#define FREQ_50HZ   20000000;   // (0.02 sec)
#define FREQ_100HZ  10000000;   // (0.01 sec)
#define FREQ_200HZ  5000000    // (.005 sec)
#define FREQ_250HZ  4000000;    // (.004 sec)
#define FREQ_500HZ  2000000;    // (.002 sec)

#include <time.h>

class Scheduler {

public:
    Scheduler (long period);
    ~Scheduler();

    void update();
    void sleep();

    timespec& getCurrentTime();
    timespec& getNextShot();

    double getPeriod();
    double getFrequency();

private:

    void normalizeTimespec(timespec* t);
    void getTime(timespec* t);

    long period;

    timespec currTime;
    timespec nextShot;

};

#endif /* SCHEDULER_H_ */
