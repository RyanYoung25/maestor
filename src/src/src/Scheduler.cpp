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
 * A Scheduler that keeps the update loop running at the specified period. 
 */

#include "Scheduler.h"

/**
 * Create the Scheduler object with a certian period
 */
Scheduler::Scheduler(long period){
    this->period = period;
    getTime(&nextShot);
    update();
}

/**
 * Destructor
 */
Scheduler::~Scheduler() {}

/**
 * update the timing of the the scheduler
 */
void Scheduler::update(){
    getTime(&currTime);

    do {
        nextShot.tv_nsec += period;
        normalizeTimespec(&nextShot);
    } while (currTime.tv_sec > nextShot.tv_sec ||
            (currTime.tv_sec == nextShot.tv_sec && currTime.tv_nsec > nextShot.tv_nsec));
}

/**
 * Sleep the loop. 
 */
void Scheduler::sleep(){
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextShot, NULL);
}

/**
 * Get the period that the scheduler is operating at
 * @return The period
 */
double Scheduler::getPeriod(){
    return period / NSEC_PER_SECOND;
}

/**
 * Get the frequency that the scheduler is operating at
 * @return The frequency
 */
double Scheduler::getFrequency(){
    return NSEC_PER_SECOND / period; 
}

/**
 * Get the current time
 * @return The current time
 */
timespec& Scheduler::getCurrentTime(){
    return currTime;
}

/**
 * Get the next shot
 * @return The next shot
 */
timespec& Scheduler::getNextShot(){
    return nextShot;
}

/**
 * Normalize the timespec 
 * @param t The timespec to normailze
 */
void Scheduler::normalizeTimespec(timespec* t){
    while (t->tv_nsec > NSEC_PER_SECOND){
        t->tv_nsec -= NSEC_PER_SECOND;
        t->tv_sec++;
    }
}

/**
 * Get the time of the timespec
 * @param t The timespec
 */
void Scheduler::getTime(timespec* t){
    clock_gettime(CLOCK_MONOTONIC, t);
}
