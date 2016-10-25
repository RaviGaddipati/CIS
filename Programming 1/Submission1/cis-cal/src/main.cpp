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

/**
 * CLI Usage information.
 */
void printusage();

/**
 * @brief
 * Prints the error between the post positions and the expected values
 * of two output files.
 * @param f1 File 1
 * @param f2 File 2
 */
void error_report(const std::string &f1, const std::string &f2);

int main(const int argc, const char *argv[]) {
    if (argc != 2) {
        printusage();
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

    const std::string fileroot(argv[1]);
    std::string filename = split(fileroot, '/').back();
    std::string calbody_file = fileroot + "-calbody.txt",
                calreadings_file = fileroot + "-calreadings.txt",
                empivot_file = fileroot + "-empivot.txt",
                optpivot_file = fileroot + "-optpivot.txt",
                output1_debug = fileroot + "-output1.txt",
                output1_file = "OUTPUT/" + filename + "-output-1.txt",
                output2_file = "OUTPUT/" + filename + "-output-2.txt";


    // Open all the files.

    cis::CalBody<double> calbody(calbody_file);
    cis::CalReadings<double> calreadings(calreadings_file);
    cis::EMPivot<double> empivot(empivot_file);
    cis::OptPivot<double> optpivot(optpivot_file);

    const Eigen::Matrix<double, 3, 1> em_post = cis::pivot_calibration(empivot.em_marker_probe()).block(3,0,3,1);
    const Eigen::Matrix<double, 3, 1> opt_post = cis::pivot_calibration_opt(optpivot.opt_marker_probe(),
                                                                            optpivot.opt_marker_embase()).block(3,0,3,1);
    std::vector<cis::PointCloud<double>> expected = cis::distortion_calibration(calbody, calreadings);

    // Write output file, generate error report if there's a debug file.
    try {
        cis::output_writer(output1_file, expected, em_post, opt_post);
    } catch (std::exception &e) {
        std::cerr << "\n\nError writing output, does OUTPUT/ directory exist?\n" << std::endl;
        throw;
    }

    // Error report
    if (file_exists(output1_debug)) {
        error_report(output1_file, output1_debug);
    }
}

/**
 * @brief
 * Prints a point as a CSV line.
 * @param os Stream to print to
 * @param p Point
 * @return os
 */
template <typename T>
std::ostream &print_point(std::ostream &os, const Eigen::Matrix<T, 3, 1> &p) {
    os << p(0) << ',' << p(1) << ',' << p(2);
    return os;
}


void error_report(const std::string &f1, const std::string &f2) {
    cis::OutputPraser<double> a(f1);
    cis::OutputPraser<double> b(f2);
    std::cerr << "\n\nComparing:\t" << a.name() << "\t" << b.name() << std::endl;

    std::cerr << "EM Post:\t";
    print_point(std::cerr, a.em_post());
    std::cerr << '\t';
    print_point(std::cerr, b.em_post());
    std::cerr << "\n\tError: ";
    print_point<double>(std::cerr, a.em_post() - b.em_post());
    std::cerr << std::endl;

    std::cerr << "Opt Post:\t";
    print_point(std::cerr, a.opt_post());
    std::cerr << '\t';
    print_point(std::cerr, b.opt_post());
    std::cerr << "\n\tError: ";
    print_point<double>(std::cerr, a.opt_post() - b.opt_post());
    std::cerr << std::endl;

    std::cerr << "\nFrame RMS error:\n";
    Eigen::Matrix<double, 3, 1> total_error = {0,0,0}, frame_err;
    for (size_t i = 0; i < a.expected().size(); ++i) {
        std::cerr << '\t' << i << ":\t";
        frame_err = a.expected().at(i).RMS_error(b.expected().at(i));
        total_error += frame_err;
        print_point(std::cerr, frame_err);
        std::cerr << '\n';
    }
    std::cerr << "\nAverage RMS error:\t";
    total_error.array() /= a.expected().size();
    print_point(std::cerr, total_error);
    std::cerr << '\n' << std::endl;
}

void printusage() {
    std::cout << "\nEN.600.465 Computer Integrated Surgery PA 1,2\nRavi Gaddipati, Doran Walsten\n"
              << __DATE__ << "\n\n"
              << "Usage:\n\tciscal <fileroot>\n\nThe following files should exist:\n\tfileroot-calbody.txt"
              << "\n\tfileroot-calreadings.txt\n\tfileroot-empivot.txt\n\tfileroot-optpivot.txt\n"
              << "Output file written to:\n\tOUTPUT/filename-output-1.txt\n\tWhere filename does not include the path.\n\n"
              << "To run test cases and exit:\n\tciscal test\n\n"
              << "This program makes use of Eigen (eigen.tuxfamily.org) and doctest (github.com/onqtam/doctest)\n"
              << std::endl;
}