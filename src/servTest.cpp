#include <vector>

#include "servTest.h"

using std::vector;

bool fib(ServiceTest::fib::Request &req, ServiceTest::fib::Response &res){
    res.res = fibn(req.num);
    return true;
}

int fibn(int n){
    if (n < 0)
        return 1;
    if (n < 2)
        return n;
    int val = memoize(n - 1, -1);
    if (val == -1)
        memoize(n - 1, fibn(n - 1));

    return fibn(n - 1) + fibn(n - 2);
}

int memoize(int n, int value){
    static vector<int> memory;
    if (memory.size() <= n)
        memory.resize(n * 2, -1);
    if (value != -1)
        memory[n] = value;
    return memory[n];
}
