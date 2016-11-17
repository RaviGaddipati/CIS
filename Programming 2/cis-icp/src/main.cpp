/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date Nov 14, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Computer Integrated Surgery
 * Programming assignment 3
 * Usage:
 *  cisicp test
 * OR:
 *  cisicp SurfaceFile PointerBodyFile FixedBodyFile SampleReadingsFile OutputFile [DebugOutput]
 *
 * @file
 */

// User controlled test execution
#define DOCTEST_CONFIG_IMPLEMENT

#include <iostream>
#include <doctest.h>
#include <algorithm>
#include "utils.h"
#include "registration.h"
#include "icp.h"

/**
 * Given two output files, compute the average error between the values.
 * @param file1
 * @param file2
 */
void compare_outputs(std::string file1, std::string file2) {
    std::ifstream a(file1);
    if (!a.good()) throw std::invalid_argument("Error opening file: " + file1);
    std::ifstream b(file2);
    if (!a.good()) throw std::invalid_argument("Error opening file: " + file2);

    // Parse meta information
    std::string line;
    std::getline(a, line);
    make_uniform(line);

    size_t Na = std::stoul(split(line, ',')[0]);
    std::getline(b, line);
    make_uniform(line);

    if (Na != std::stoul(split(line, ',')[0])) {
        throw std::invalid_argument("Error: Different number of points in output files.");
    }

    std::string line2;
    double d_err = 0, c_err = 0, diff_err = 0;
    while (std::getline(a, line) && std::getline(b, line2)) {
        make_uniform(line);
        make_uniform(line2);

        const auto arec = split(line, ',');
        const auto brec = split(line2, ',');

        if (arec.size() != 7 || brec.size() != 7)
            throw std::invalid_argument("Invalid line format in output file:\n\t" + line + "\n\t" + line2);

        std::vector<std::string> ad(arec.begin(), arec.begin() + 3);
        std::vector<std::string> bd(brec.begin(), brec.begin() + 3);
        std::vector<std::string> ac(arec.begin() + 3, arec.begin() + 6);
        std::vector<std::string> bc(arec.begin() + 3, arec.begin() + 6);
        double a_diff = std::stod(arec[6]);
        double b_diff = std::stof(brec[6]);

        if (ad.size() != 3 || bd.size() != 3 || ac.size() != 3 || bc.size() != 3)
            throw std::invalid_argument("Malformed point in lines:\n\t" + line + "\n\t" + line2);

        const cis::Point ad_p = {std::stod(ad[0]), std::stod(ad[1]), std::stod(ad[2])};
        const cis::Point bd_p = {std::stod(bd[0]), std::stod(bd[1]), std::stod(bd[2])};
        const cis::Point ac_p = {std::stod(ac[0]), std::stod(ac[1]), std::stod(ac[2])};
        const cis::Point bc_p = {std::stod(bc[0]), std::stod(bc[1]), std::stod(bc[2])};

        d_err += (ad_p - bd_p).norm();
        c_err += (ac_p - bc_p).norm();
        diff_err += std::abs(a_diff - b_diff);
    }

    std::cout << "Comparing \"" << file1 << "\" and \"" << file2 << "\"\n";
    std::cout << "\nd_k average error: " << d_err/Na << "\n"
              << "c_k average error: " << c_err/Na << "\n"
              << "Difference average error: " << diff_err/Na << "\n" << std::endl;
}

void find_closest(const int argc, const char *argv[]) {
    //Want the files which correspond to the rigid bodies
    const std::string surface_file = argv[1],
            rigidbodyA_file = argv[2],
            rigidbodyB_file = argv[3],
            sample_file = argv[4],
            output_file = argv[5];

    //Open all of the files
    cis::RigidBody bodyA(rigidbodyA_file);
    cis::RigidBody bodyB(rigidbodyB_file);
    cis::SurfaceFile sur(surface_file);

    //Given the sample readings file, compute the transformation on each frame
    cis::SampleReadings samples(sample_file,bodyA.markers().size(),bodyB.markers().size());

    //Compute the point cloud of tip coordinates with respect to the fixed rigid body
    cis::PointCloud d = cis::pointer_to_fixed(bodyA, bodyB, samples.pointer_rigid_body(), samples.fixed_rigid_body());

    cis::PointCloud closest_points;
    for (size_t p = 0; p < d.size(); ++p) {
        closest_points.add_point(cis::project_onto_surface_naive(d.at(p), sur));
    }

    std::ofstream o(output_file);
    if (!o.good()) throw std::invalid_argument("Error opening output file: " + output_file);
    o.setf(std::ios::fixed, std::ios::floatfield);;
    // Print output
    o << d.size() << ',' << output_file << '\n';
    for (size_t i = 0; i < d.size(); ++i) {
        o.precision(2);
        print_point(o, d.at(i));
        o << '\t';
        print_point(o, closest_points.at(i));
        o << '\t';
        o.precision(3);
        o << (d.at(i) - closest_points.at(i)).norm() << '\n';
    }
    o.close();

    // Output error
    if (argc == 7) {
        compare_outputs(output_file, argv[6]);
    }
}

int main(const int argc, const char *argv[]) {
    // Run test cases
    if (argc > 1 && !strcmp(argv[1], "test")) {
        doctest::Context doc(argc, argv);
        doc.setOption("no-breaks", true);
        doc.setOption("abort-after", 10);
        doc.setOption("sort", "name");
        return doc.run();
    }

    if (argc < 6) {
        std::cerr << "\nUsage:\n"
                  << "cisicp SurfaceFile PointerBodyFile FixedBodyFile SampleReadingsFile OutputFile [DebugOutput]\n"
                  << "\"cisicp test\" to run test cases.\n"
                  << std::endl;
        exit(0);
    }

    try {
        find_closest(argc, argv);
    } catch (std::exception &e) {
        std::cerr << "\nFatal Error!\n\t" << e.what() << '\n' << std::endl;
        return 1;
    }

    return 0;
}