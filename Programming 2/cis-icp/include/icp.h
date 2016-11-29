/**
 * @author Ravi Gaddipati
 * @date Nov 3, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Iterative closest point.
 *
 * @file
 */

#ifndef CIS_ICP_ICP_H
#define CIS_ICP_ICP_H

#include <iostream>
#include "utils.h"
#include "files.h"
#include "surface.h"

namespace cis {

    /**
     * @brief
     * Given a point, find the closest point on a mesh. The naive method linearly searches
     * through all triangles.
     * @param p Point to project
     * @param surface to project onto
     * @return Point on surface
     */
    Point project_onto_surface_naive(const Point &p, const SurfaceFile &surface);

    /**
     * Given a point cloud of feature points, performs ICP algorithm to determine the best registration transformation
     * between the feature point space and the surface space
     * @param q Original point cloud of feature points
     * @param surfaceFile Surface file to register to
     * @return The estimated registration transformation following ICP
     */
    Eigen::Transform<double, 3, Eigen::Affine> icp(const PointCloud &q, const SurfaceFile &surfaceFile);

    Point project_onto_surface_kdtree(const Point &p, std::shared_ptr<Surface::Division> root);

    inline Point project_onto_surface_kdtree(const Point &p, const Surface &s) {
        return project_onto_surface_kdtree(p, s.root());
    }

}

TEST_CASE ("Segment Projection") {
    cis::Point p(0,0,0);
    cis::Point q(1,0,0);

    cis::Point c(1,1,1);
    cis::Point check;

    check << 1,0,0;
            CHECK(cis::project_onto_segment(c,p,q) == check);
    c << -1,-1,-1;
    check << 0,0,0;
            CHECK(cis::project_onto_segment(c,p,q) == check);
    c << 0.5,0,0;
    check << 0.5,0,0;
            CHECK(cis::project_onto_segment(c,p,q) == check);
    c << 0.5,0.5,0.5;
    check << 0.5,0,0;
            CHECK(cis::project_onto_segment(c,p,q) == check);
    q << 1,1,1;
    check << 0.5,0.5,0.5;
            CHECK(cis::project_onto_segment(c,p,q) == check);
}

TEST_CASE ("Triangle Projection") {
    // Define a triangle
    cis::Point triangle[3];
    triangle[0] << 1,1,0;
    triangle[1] << 4,2,0;
    triangle[2] << 2,3,0;

    cis::Point P;
            SUBCASE("Inside triangle") {
        P << 2,2,2;
        auto prime = cis::project_onto_triangle(P, triangle[0], triangle[1], triangle[2]);
                CHECK(prime(0) == 2);
                CHECK(prime(1) == 2);
                CHECK(prime(2) == 0);
    }

            SUBCASE("Vertex 1") {
        P << 0,0,2;
        auto prime = cis::project_onto_triangle(P, triangle[0], triangle[1], triangle[2]);
                CHECK(prime(0) == 1);
                CHECK(prime(1) == 1);
                CHECK(prime(2) == 0);
    }

            SUBCASE("Vertex 2") {
        P << 5,2,2;
        auto prime = cis::project_onto_triangle(P, triangle[0], triangle[1], triangle[2]);
                CHECK(prime(0) == 4);
                CHECK(prime(1) == 2);
                CHECK(prime(2) == 0);
    }

            SUBCASE("Vertex 3") {
        P << 2,5,2;
        auto prime = cis::project_onto_triangle(P, triangle[0], triangle[1], triangle[2]);
                CHECK(prime(0) == 2);
                CHECK(prime(1) == 3);
                CHECK(prime(2) == 0);
    }

            SUBCASE("Top Edge") {
        P << 3,4,0;
        auto prime = cis::project_onto_triangle(P, triangle[0], triangle[1], triangle[2]);
                CHECK(approx_eq(prime(0), 2.4));
                CHECK(approx_eq(prime(1), 2.8));
                CHECK(prime(2) == 0);
    }

            SUBCASE("Bottom Edge") {
        P << 2,0,0;
        auto prime = cis::project_onto_triangle(P, triangle[0], triangle[1], triangle[2]);
                CHECK(approx_eq(prime(0), 1.6));
                CHECK(approx_eq(prime(1), 1.2));
                CHECK(prime(2) == 0);
    }

            SUBCASE("Left Edge") {
        P << 0,2,0;
        auto prime = cis::project_onto_triangle(P, triangle[0], triangle[1], triangle[2]);
                CHECK(approx_eq(prime(0), 1.2));
                CHECK(approx_eq(prime(1), 1.4));
                CHECK(prime(2) == 0);
    }

            SUBCASE("Project directly onto vertex") {
        P << 1,1,2;
        auto prime = cis::project_onto_triangle(P, triangle[0], triangle[1], triangle[2]);
                CHECK(prime(0) == 1);
                CHECK(prime(1) == 1);
                CHECK(prime(2) == 0);
    }

            SUBCASE("Project directly onto edge") {
        P << 2.5,1.5,2;
        auto prime = cis::project_onto_triangle(P, triangle[0], triangle[1], triangle[2]);
                CHECK(prime(0) == 2.5);
                CHECK(prime(1) == 1.5);
                CHECK(prime(2) == 0);
    }
}

TEST_CASE("Project To Surface") {
    /**
     * Test case is 3 triangles on the X, Y, and Z origin planes (2,3,5) triangles with common
     * origin vertex.
     * Point is moved around to make sure correct point and point on triangle is used.
     */
    const std::string tmpfile = "cis-icp-doctest.tmp";
    {
        std::ofstream o(tmpfile);
        o << "7\n"
          << "0 0 0\n"
          << "2 0 0\n"
          << "3 0 0\n"
          << "0 2 0\n"
          << "0 3 0\n"
          << "0 0 2\n"
          << "0 0 3\n"
          << "3\n"
          << "0 2 3 -1 -1 -1\n"
          << "0 1 6 -1 -1 -1\n"
          << "0 4 5 -1 -1 -1\n";
    }

    cis::SurfaceFile bs(tmpfile);
            REQUIRE(bs.triangles().rows() == 3);
            REQUIRE(bs.vertices().size() == 7);

    // 1,1,1 is equidistant to all triangles
    cis::Point p , prj, expect;

    p << 1,1,0.5;
    expect << 1,1,0;
    prj = cis::project_onto_surface_naive(p, bs);
            CHECK(prj == expect);

    p << 0.5,1,1;
    expect << 0,1,1;
    prj = cis::project_onto_surface_naive(p, bs);
            CHECK(prj == expect);

    p << 1,0.5,1;
    expect << 1,0,1;
    prj = cis::project_onto_surface_naive(p, bs);
            CHECK(prj == expect);

    p << -1,-1,-1;
    expect << 0,0,0;
    prj = cis::project_onto_surface_naive(p, bs);
            CHECK(prj == expect);

    p << 4,0,0;
    expect << 03,0,0;
    prj = cis::project_onto_surface_naive(p, bs);
            CHECK(prj == expect);

    p << 0,4,0;
    expect << 0,3,0;
    prj = cis::project_onto_surface_naive(p, bs);
            CHECK(prj == expect);

    p << 0,0,4;
    expect << 0,0,3;
    prj = cis::project_onto_surface_naive(p, bs);
            CHECK(prj == expect);

    remove(tmpfile.c_str());
}

#endif //CIS_ICP_ICP_H
