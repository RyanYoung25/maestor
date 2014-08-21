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
