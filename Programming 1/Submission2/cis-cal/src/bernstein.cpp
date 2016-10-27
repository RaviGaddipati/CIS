/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date October 23, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * scales a point to (0,1)
 *
 * @file
 */

#include "bernstein.h"

cis::Point cis::scale_to_box(Point q,
                        Point q_min,
                        Point q_max) {
    Point u = {0, 0, 0};
    u(0) = (q(0) - q_min(0)) / (q_max(0) - q_min(0));
    u(1) = (q(1) - q_min(1)) / (q_max(1) - q_min(1));
    u(2) = (q(2) - q_min(2)) / (q_max(2) - q_min(2));
    return u;
}
