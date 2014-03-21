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
#define FREQ_200HZ  5000000     // (.005 sec)
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

private:

    void normalizeTimespec(timespec* t);
    void getTime(timespec* t);

    long period;

    timespec currTime;
    timespec nextShot;

};

#endif /* SCHEDULER_H_ */
