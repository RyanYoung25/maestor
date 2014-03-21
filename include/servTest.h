
#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "ServiceTest/fib.h"

int fibn(int n);
int memoize(int n, int value);

bool fib(ServiceTest::fib::Request &req, ServiceTest::fib::Response &res);
