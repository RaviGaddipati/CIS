//
// Created by Doran W on 10/25/16.
//

#ifndef CIS_CAL_BERNSTEIN_H
#define CIS_CAL_BERNSTEIN_H

//#include "pointcloud.h"
//#include "utils.h"
#include "Eigen"
#include "Eigenvalues"

namespace cis {


    using Point = PointCloud<double>::Point;

    /**
     * Given degree n, step k, and input u computes the bernstein basis polynomial
     * @param k - Range 0 - n
     * @param n - Degree of polynomial
     * @param u - input
     * @return Value of the basis polynomial given input u
     */
    double bernstein(int k, int n, double u) {
        return nChoosek(n, k) * pow(u, k) * pow(1 - u, n - k);
    }

    /**
     * Used to scale point q to be constrained to 0 - 1 on all entries x,y,z
     * @param q - Original point
     * @param q_min - Defines the minimum value in each direction
     * @param q_max - Defines the maximum value in each direction
     * @return The newly created point that is now bounded
     */
    Point scaleToBox(Point q, Point q_min, Point q_max) {
        Point u = {0, 0, 0};
        u(0) = (q(0) - q_min(0)) / (q_max(0) - q_min(0));
        u(1) = (q(1) - q_min(1)) / (q_max(1) - q_min(1));
        u(2) = (q(2) - q_min(2)) / (q_max(2) - q_min(2));

        return u;
    }

    /**
     * Compute the interpolation polynomial using 5th degree Bernstein polynomials for the provided point u
     * @param u - Input Point, must be bounded between 0 - 1 for all coordinates
     * @return Vector representing the interpolation polynomial F_000 ... F_555
     */
    Eigen::Matrix<double, 1, 125> F(Point u) {
        Eigen::Matrix<double, 1, 125> F = Eigen::Matrix<double, 1, 125>::Zero(); //Initialize to 0
        int counter = 0;
        for (int i = 0; i <= 5; i++) {
            double B_i = bernstein(i, 5, u(0));
            for (int j = 0; j <= 5; j++) {
                double B_j = bernstein(j, 5, u(1));
                for (int k = 0; k <= 5; k++) {
                    double B_k = bernstein(k, 5, u(2));
                    F(counter) = B_i * B_j * B_k;
                }
            }
        }
        return F;
    }
}

TEST_CASE("Bernstein Test Cases") {
    using namespace cis;
    PointCloud<double> pc1{{{1, 2, 3}, {0, 1, .5}, {0, 0, 1}, {-1, 0, 0}, {3, 8, 7}}};
    cis::Point q_min = {-1, 0, 0};
    cis::Point q_max = {3,8,7};
    bool norm = true;
    for(int i = 0; i < pc1.size(); i++) {
        cis::Point u = scaleToBox(pc1.at(i),q_min,q_max);
        norm = (u(0) >= 0 && u(0) <= 1) && (u(1) >= 0 && u(1) <= 1) && (u(2) >= 0 && u(2) <= 1);
    }
    CHECK(norm); //Ensure that the scaleToBox method is working

    //Check that nChoosek is working
    CHECK(nChoosek(5,-1) == 0);
    CHECK(nChoosek(5,0) == 1);
    CHECK(nChoosek(5,1) == 5);
    CHECK(nChoosek(5,2) == 10);
    CHECK(nChoosek(5,3) == 10);
    CHECK(nChoosek(5,4) == 5);
    CHECK(nChoosek(5,5) == 1);
    CHECK(nChoosek(5,6) == 0);

    //Check that Bernstein base polynomial is working
    double x = 0.5;
    //Checks inspired by rules on Wikipedia for Bernstein polynomial expansion: https://en.wikipedia.org/wiki/Bernstein_polynomial
    CHECK(bernstein(0,0,x) == 1);
    CHECK(bernstein(1,2,x) == 2*x*(1-x));
    CHECK(bernstein(0,4,x) == pow(1-x,4));
    CHECK(bernstein(2,3,x) == 3*pow(x,2)*(1-x));
}



#endif //CIS_CAL_BERNSTEIN_H
