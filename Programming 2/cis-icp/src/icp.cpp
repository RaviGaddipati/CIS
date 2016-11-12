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


cis::Point cis::project_onto_segment(const cis::Point &c, const cis::Point &p, const cis::Point &q) {
    const Point qmp = q - p;
    return p + ( max(0.0, min((c-p).dot(qmp) / qmp.dot(qmp), 1.0)) * qmp );
}

cis::Point cis::project_onto_triangle(const cis::Point &p,
                                      const cis::Point &v1, const cis::Point &v2, const cis::Point &v3) {
    // Define triangle as vertex and two edges
    const Eigen::Vector3d u = v2 - v1,
            v = v3 - v1,
            w = p - v1,
            n = u.cross(v);
    const double inv_n_sq = 1.0 / n.dot(n);

    // Barycentric coords
    const double b2 = (u.cross(w)).dot(n) * inv_n_sq,
            b1 = (w.cross(v)).dot(n) * inv_n_sq,
            b0 = 1 - b1 - b2;

    // If all barycentric coords are >= 0, inside triangle / on edge
    if (b2 >= 0 && b1 >= 0 && b0 >= 0) return Point((v1 * b0) + (v2 * b1) + (v3 * b2));

    // Lies on opposite side of v1
    if (b0 <= 0) {
        if (b2 <= 0) return v2;
        if (b1 <= 0) return v3;
        return project_onto_segment(p, v2, v3);
    }

    // Lies on opposite side of v2
    if (b1 <= 0) {
        if (b2 <= 0) return v1;
        return project_onto_segment(p, v1, v3);
    }

    // Lies on opposite side of v3
    assert(b2 <= 0 && "Default case should always be true if previous projections are not");
    return project_onto_segment(p, v1, v2);
}

cis::Point cis::project_onto_surface_naive(const Point &p, const Surface &surface) {
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