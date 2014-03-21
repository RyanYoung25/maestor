/*
 * Singleton.h
 *
 *  Created on: Oct 15, 2013
 *      Author: maestro
 */

#ifndef SINGLETON_H_
#define SINGLETON_H_

template <typename T>
class Singleton {
public:
    static T* instance(){
        static T singleton;
        return &singleton;
    }
};

#endif /* SINGLETON_H_ */
