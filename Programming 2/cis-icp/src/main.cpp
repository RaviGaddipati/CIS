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
 *  cisicp fileroot
 * See printed usage for more info.
 *
 * @file
 */

// User controlled test execution
#define DOCTEST_CONFIG_IMPLEMENT

#include <iostream>
#include <doctest.h>
#include "main.h"
#include "utils.h"
#include "registration.h"
#include "icp.h"
#include "surface.h"





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

    const std::string fileroot(argv[1]);
    const std::string filename = split(fileroot, '/').back();

    //Want the files which correspond to the rigid bodies
    const std::string rigidbodyA_file = "INPUT/Problem3-BodyA.txt";
    const std::string rigidbodyB_file = "INPUT/Problem3-BodyB.txt";

    //Want the debug file specific sample data
    const std::string sample_file = fileroot + "-SampleReadingsTest.txt",
            output1_file = "OUTPUT/" + filename + "-Output.txt";

    //Open all of the files
    cis::RigidBody bodyA(rigidbodyA_file);
    cis::RigidBody bodyB(rigidbodyB_file);

    //Given the sample readings file, compute the transformation on each frame
    cis::SampleReadings samples(sample_file,bodyA.size(),bodyB.size());

    //Compute the point cloud of tip coordinates with respect to the fixed rigid body
    cis::PointCloud d = cis::pointer_to_fixed(bodyA,bodyB,samples.pointer_rigid_body(),samples.fixed_rigid_body());

    std::cout << d << std::endl;




}