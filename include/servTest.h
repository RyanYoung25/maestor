
#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "maestor/fib.h"

int fibn(int n);
int memoize(int n, int value);

bool fib(maestor::fib::Request &req, maestor::fib::Response &res);
