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

#include "pointcloud.h"
#include "utils.h"

namespace cis {

    /**
     * @brief
     * Project the point c onto line with endpoints p,q. Returns the distance
     * from point p along vector (q-p). The projected coordinates would then be
     * p + returned_value * (q - p)
     * @param c Point to project
     * @param p endpoint 1
     * @param q endpoint 2
     * @return distance from p, constrained to line segment.
     */
    double project_onto_segment(const Point &c, const Point &p, const Point &q);

    /**
     * @brief
     * Computes the closest point on the triangle, including the edges to point P.
     * @param p Projects this point onto triangle formed by verticies v1,v2,v3
     * @param v1 Vertex 1
     * @param v2 Vertex 2
     * @param v3 Vertex 3
     * @return Point on triangle closest to p
     */
    Point project_onto_triangle(const Point &p,
                                const Point &v1, const Point &v2, const Point &v3);
}

#endif //CIS_ICP_ICP_H
