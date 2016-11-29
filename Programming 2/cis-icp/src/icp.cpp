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

#include <surface.h>
#include "icp.h"


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


Eigen::Transform<double, 3, Eigen::Affine> cis::icp(const PointCloud &q, const SurfaceFile &surfaceFile) {
    //Generate the root surface to match to
    cis::Surface surface(surfaceFile.cat_triangles(), surfaceFile.neighbor_triangles());

    //Initialize all relevant parameters
    //eta (learning rate)
    //sigma (standard dev of error)
    //max_error
    //mean_error

    //Initialize the registration transformation as just an identity matrix
    Eigen::Transform<double, 3, Eigen::Affine> F_reg(
            Eigen::Translation<double, 3>(0, 0, 0) *
            Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX())
    );

    //Hold whether terminating condition is satisfied
    bool term = true;

    while (!term) {
        cis::PointCloud A();
        cis::PointCloud B();

        for (size_t p = 0; p < q.size(); ++p) {
            //Use project_onto_surface_kd given F_reg

            //if difference is below threshold eta, add original point to A, closest point to B
        }

        //Use Horn method to find new best transformation from A to B

        //Update parameters (see slides)

        //if termination condition is met, update term

    }

    return F_reg;
}

cis::Point cis::project_onto_surface_kdtree(const cis::Point &p,std::shared_ptr<cis::Surface::Division> root) {

}
