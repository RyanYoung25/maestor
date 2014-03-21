#include <iostream>

#include "ros/ros.h"

#include "servTest.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "ServiceTest"); //Init the node
    ros::NodeHandle n; //Fully initializes the node

    ros::ServiceServer srv = n.advertiseService("fib", &fib);

    while (ros::ok()) {
        ros::spin();
    }
    return 0;
}

