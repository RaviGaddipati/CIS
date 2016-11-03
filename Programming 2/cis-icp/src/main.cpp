/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date Nov 2, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Computer Integrated Surgery
 * Programming assignment 1,2
 * Usage:
 *  ciscal test
 * OR:
 *  ciscal fileroot
 * See printed usage for more info.
 *
 * @file
 */

// User controlled test execution
#define DOCTEST_CONFIG_IMPLEMENT

#include <iostream>
#include <doctest.h>
#include "main.h"
#include "files.h"
#include "icp.h"

int main(const int argc, const char *argv[]) {
    if (argc != 2) {
        return 0;
    }

    // Run test cases
    if (!strcmp(argv[1], "test")) {
        doctest::Context doc(argc, argv);
        doc.setOption("no-breaks", true);
        doc.setOption("abort-after", 10);
        doc.setOption("sort", "name");
        return doc.run();
    }
}