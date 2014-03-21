#include <sched.h>
#include <sys/io.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <math.h>

#include "Scheduler.h"
#include "servTest.h"

using ros::NodeHandle;
using ros::ServiceServer;
using ros::init;

void setRealtime();

using std::cout;
using std::endl;

int main(int argc, char **argv) {
    setRealtime();

    init(argc, argv, "Maestor"); //Init the node
    NodeHandle n; //Fully initializes the node
    Scheduler timer(FREQ_200HZ);

    RobotControl robot(n, timer);

    ServiceServer srv = n.advertiseService("fib", &fib);

    while (ros::ok()) {
        ros::spin();

        robot.updateHook();

        timer.sleep();
        timer.update();
    }
    return 0;
}

void setRealtime(){
    // Taken from hubo-ach/src/hubo-daemonizer.c

    struct sched_param param;

    // Declare ourself as a real time task
    param.sched_priority = 49;
    if(sched_setscheduler(getpid(), SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
        exit(-1);
    }
}
