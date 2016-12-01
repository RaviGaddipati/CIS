/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date Nov 2, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Iterative closest point implementation.
 *
 * @file
 */

#include "files.h"
#include "horn.h"
#include <queue>
#include "icp.h"
#include "doctest.h"


cis::Point cis::project_onto_surface_naive(const Point &p, const SurfaceFile &surface) {
    auto &vert = surface.vertices();
    auto &triangles = surface.triangles();
    Point projected, min_point;
    double min_norm = std::numeric_limits<double>::max(), curr_norm;

    for (size_t i = 0; i < triangles.rows(); ++i) {
        const auto &idx = triangles.row(i);
        projected = project_onto_triangle(p, vert.at(idx(0)), vert.at(idx(1)), vert.at(idx(2)));
        curr_norm = (projected - p).norm();
        if (curr_norm < min_norm) {
            min_norm = curr_norm;
            min_point = projected;
        }
    }
    return min_point;
}


Eigen::Transform<double, 3, Eigen::Affine> cis::icp(const PointCloud &q, SurfaceFile &surfaceFile) {
    //Generate the root surface to match to
    cis::Surface surface = surfaceFile.surface();

    //Initialize all relevant parameters
    double eta = 200; //Generous initial bound
    double gamma = 0.96; //Threshold for the terminating condition

        //Initialize the registration transformation as just an identity matrix
        Eigen::Transform<double, 3, Eigen::Affine> F_reg(
        Eigen::Translation<double, 3>(q.centroid())
        );

        //Hold whether terminating condition is satisfied
        bool term = false;

        std::vector<Point> A_pts, B_pts;
        cis::Point c_k, d, e, Fq;
        double sig, mean;

        std::queue<double> err;
        for (size_t i = 0; i < 5; ++i) err.push(std::numeric_limits<double>::max());

        while (!term) {
            cis::PointCloud A, B;
            A_pts.clear();
            B_pts.clear();

            for (size_t p = 0; p < q.size(); ++p) {
                //Use project_onto_surface_kd given F_reg

                Fq = F_reg * q.at(p);
                #ifdef CIS_ICP_USE_NAIVE
                c_k = project_onto_surface_naive(Fq, surfaceFile);
                #else
                c_k = surface.root()->find_closest_point(Fq);
                #endif



                d = c_k - Fq;

                //If valid pair, add to the clouds above
                if (d.norm() < eta) {
                    A_pts.push_back(q.at(p));
                    B_pts.push_back(c_k);
                }
            }

            A.add_points(A_pts);
            B.add_points(B_pts);

            //Use Horn method to find new best transformation from A to B
            F_reg = cloud_to_cloud(A, B);

            //Determine error and update eta
            sig = 0;
            mean = 0;
            for (size_t i = 0; i < A.size(); i++) {
                e = B.at(i) - F_reg * A.at(i);
                sig = sig + e.squaredNorm();
                mean = mean + e.norm();
            }
            //  sigma.push_back(sqrt(sig)/A.size());
            err.push(mean / A.size());

            eta = 8 * err.back(); //Maybe this is the last element? May have to confirm
            //if termination condition is met, update term
            const double crit = err.back() / err.front();
            err.pop();
            term = (gamma <= crit && crit <= 1);
        }


    return F_reg;
}

cis::Point cis::project_onto_surface_kdtree(const cis::Point &p,std::shared_ptr<cis::Surface::Division> root) {
    return root->find_closest_point(p);
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
    cis::Surface sur(bs.cat_triangles(), bs.neighbor_triangles());
    REQUIRE(bs.triangles().rows() == 3);
    REQUIRE(bs.vertices().size() == 7);

    // 1,1,1 is equidistant to all triangles
    cis::Point p , prj, expect;

    p << 1,1,0.5;
    expect << 1,1,0;
    prj = cis::project_onto_surface_naive(p, bs);
    CHECK(prj == expect);
    CHECK(prj == sur.root()->find_closest_point(p));

    p << 0.5,1,1;
    expect << 0,1,1;
    prj = cis::project_onto_surface_naive(p, bs);
    CHECK(prj == expect);
    CHECK(prj == sur.root()->find_closest_point(p));

    p << 1,0.5,1;
    expect << 1,0,1;
    prj = cis::project_onto_surface_naive(p, bs);
    CHECK(prj == expect);
    CHECK(prj == sur.root()->find_closest_point(p));

    p << -1,-1,-1;
    expect << 0,0,0;
    prj = cis::project_onto_surface_naive(p, bs);
    CHECK(prj == expect);
    CHECK(prj == sur.root()->find_closest_point(p));

    p << 4,0,0;
    expect << 03,0,0;
    prj = cis::project_onto_surface_naive(p, bs);
    CHECK(prj == expect);
    CHECK(prj == sur.root()->find_closest_point(p));

    p << 0,4,0;
    expect << 0,3,0;
    prj = cis::project_onto_surface_naive(p, bs);
    CHECK(prj == expect);
    CHECK(prj == sur.root()->find_closest_point(p));

    p << 0,0,4;
    expect << 0,0,3;
    prj = cis::project_onto_surface_naive(p, bs);
    CHECK(prj == expect);
    CHECK(prj == sur.root()->find_closest_point(p));

    remove(tmpfile.c_str());
}