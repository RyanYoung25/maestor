/*
 * Scheduler.cpp
 *
 *  Created on: Dec 17, 2013
 *      Author: solisknight
 */

#include "Scheduler.h"

Scheduler::Scheduler(long period){
    this->period = period;
    getTime(&nextShot);
    update();
}

Scheduler::~Scheduler() {}

void Scheduler::update(){
    getTime(&currTime);

    do {
        nextShot.tv_nsec += period;
        normalizeTimespec(&nextShot);
    } while (currTime.tv_sec > nextShot.tv_sec ||
            (currTime.tv_sec == nextShot.tv_sec && currTime.tv_nsec > nextShot.tv_nsec));
}

void Scheduler::sleep(){
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextShot, NULL);
}

double Scheduler::getPeriod(){
    return period / NSEC_PER_SECOND;
}

double Scheduler::getFrequency(){
    return NSEC_PER_SECOND / period; 
}

timespec& Scheduler::getCurrentTime(){
    return currTime;
}

timespec& Scheduler::getNextShot(){
    return nextShot;
}

void Scheduler::normalizeTimespec(timespec* t){
    while (t->tv_nsec > NSEC_PER_SECOND){
        t->tv_nsec -= NSEC_PER_SECOND;
        t->tv_sec++;
    }
}

void Scheduler::getTime(timespec* t){
    clock_gettime(CLOCK_MONOTONIC, t);
}
