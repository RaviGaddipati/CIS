#define DOCTEST_CONFIG_IMPLEMENT // User controlled test execution

#include <iostream>
#include <doctest.h>
//#include "distortion_calibration.h"
#include "pivot_calibration.h"
#include "pointcloud.h"
#include "files.h"
#include "horn.h"

int main(const int argc, const char *argv[]) {
    if (argc > 1) {
        if (!strcmp(argv[1], "test")) {
            doctest::Context doc(argc, argv);
            doc.setOption("no-breaks", true);
            doc.setOption("abort-after", 10);
            doc.setOption("sort", "name");
            return doc.run();
        }
    }

    return 0;
}