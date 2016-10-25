/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date October 23, 2016
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


#define DOCTEST_CONFIG_IMPLEMENT // User controlled test execution

#include <iostream>
#include <doctest.h>
#include "pivot_calibration.h"
#include "pointcloud.h"
#include "files.h"
#include "distortion_calibration.h"


void printusage();

int main(const int argc, const char *argv[]) {
    if (argc != 2) {
        printusage();
        return 0;
    }

    if (!strcmp(argv[1], "test")) {
        doctest::Context doc(argc, argv);
        doc.setOption("no-breaks", true);
        doc.setOption("abort-after", 10);
        doc.setOption("sort", "name");
        return doc.run();
    }

    const std::string fileroot(argv[1]);
    std::string calbody_file = fileroot + "-calbody.txt",
                calreadings_file = fileroot + "-calreadings.txt",
                empivot_file = fileroot + "-empivot.txt",
                optpivot_file = fileroot + "-optpivot.txt",
                output1_file = "OUTPUT/" + fileroot + "-output-1.txt",
                output2_file = "OUTPUT/" + fileroot + "-output-2.txt";
    if (!(file_exists(calbody_file) && file_exists(calreadings_file) &&
            file_exists(empivot_file) && file_exists(optpivot_file))) {
        std::cerr << "All required files are not present!" << std::endl;
        printusage();
        return 1;
    }

    cis::CalBody<double> calbody(calbody_file);
    cis::CalReadings<double> calreadings(calreadings_file);
    cis::EMPivot<double> empivot(empivot_file);
    cis::OptPivot<double> optpivot(optpivot_file);

    const Eigen::Matrix<double, 3, 1> em_post = cis::pivot_calibration(empivot.em_marker_probe()).block(3,0,3,1);
    const Eigen::Matrix<double, 3, 1> opt_post = cis::pivot_calibration_opt(optpivot.opt_marker_probe(), optpivot.opt_marker_embase()).block(3,0,3,1);
        auto expected = cis::distortion_calibration(calbody, calreadings);

    cis::output_writer(std::cout, output1_file, expected, em_post, opt_post);
    std::cout << em_post << '\n' << opt_post << std::endl;

}

void error_report(std::ostream &os, std::istream &is1, std::istream &is2, const bool tex=false) {

}

void printusage() {
    std::cout << "\nEN.600.465 Computer Integrated Surgery PA 1,2\nRavi Gaddipati, Doran Walsten\n"
              << __DATE__ << "\n\n"
              << "Usage:\n\tciscal <fileroot>\n\nThe following files should exist:\n\tfileroot-calbody.txt"
              << "\n\tfileroot-calreadings.txt\n\tfileroot-empivot.txt\n\tfileroot-optpivot.txt\n"
              << "Output file written to:\n\tOUTPUT/fileroot-output-1.txt\n\n"
              << "To run test cases and exit:\n\tciscal test\n\n"
              << "This program makes use of Eigen (eigen.tuxfamily.org) and doctest (github.com/onqtam/doctest)\n"
              << std::endl;
}