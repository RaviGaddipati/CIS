/**
 * @author Ravi Gaddipati, Doran Walsten
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

// Enable naive closest point search
// #define CIS_ICP_USE_NAIVE

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
    Eigen::Transform<double, 3, Eigen::Affine> icp(const PointCloud &q, SurfaceFile &surfaceFile);

    Point project_onto_surface_kdtree(const Point &p, std::shared_ptr<Surface::Division> root);

    inline Point project_onto_surface_kdtree(const Point &p, const Surface &s) {
        return project_onto_surface_kdtree(p, s.root());
    }

}

#endif //CIS_ICP_ICP_H
