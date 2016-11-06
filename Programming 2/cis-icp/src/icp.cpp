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

#include "icp.h"

double ::cis::project_onto_segment(const cis::Point &c, const cis::Point &p, const cis::Point &q) {
    const Point qmp = q-p;
    return max(0.0, min((c-p).dot(qmp) / qmp.dot(qmp), 1.0));
}

cis::Point cis::project_onto_triangle(const cis::Point &p, const cis::Point &v1, const cis::Point &v2,
                                       const cis::Point &v3) {
    // Define triangle as vertex and two edges
    const Eigen::Vector3d u = v2 - v1;
    const Eigen::Vector3d v = v3 - v1;
    const Eigen::Vector3d w = p - v1;
    const Eigen::Vector3d n = u.cross(v);
    const double inv_n_sq = 1.0 / n.dot(n);

    // Barycentric coords
    const double b2 = (u.cross(w)).dot(n) * inv_n_sq;
    const double b1 = (w.cross(v)).dot(n) * inv_n_sq;
    const double b0 = 1 - b1 - b2;

    // Outside the triangle
    if ( (!between(b0, 0, 1)) ||
         (!between(b1, 0, 1)) ||
         (!between(b2, 0, 1)) ) {
        // Projections onto all line segments
        const double p12 = project_onto_segment(p, v1, v2);
        const double p13 = project_onto_segment(p, v1, v3);
        const double p23 = project_onto_segment(p, v2, v3);

        // See if any vertex is the correct point
        if (p12 == 0 && p13 == 0) return v1;
        if (p12 == 1 && p23 == 0) return v2;
        if (p13 == 1 && p23 == 1) return v3;

        // Return the projection where the point falls on a line segment.
        if (between<false>(p12, 0, 1)) return v1 + (p12 * (v1 - v2));
        if (between<false>(p13, 0, 1)) return v1 + (p13 * (v1 - v3));
        assert(between<false>(p23, 0, 1) && "This condtion should always be true if previous ones are not.");
        return v1 + (p13 * (v2 - v3));
    }

    return Point((v1 * b0) + (v2 * b1) + (v3 * b2));
}
