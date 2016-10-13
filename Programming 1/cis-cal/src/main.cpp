#include <iostream>
#include "pointcloud.h"

#define DOCTEST_CONFIG_IMPLEMENT // User controlled test execution


int main() {
    std::cout << "Hello, World!" << std::endl;
    PointCloud<> p;
    PointCloud<>::Point t;
    p.add_point(t);
    return 0;
}