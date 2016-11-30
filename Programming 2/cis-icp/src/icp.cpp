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
#include <horn.h>
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
    double eta = 200; //Generous initial bound
    double gamma = 0.95; //Threshold for the terminating condition
    std::vector<double> sigma;
    std::vector<double> mean_error;

    //Initialize the registration transformation as just an identity matrix
    Eigen::Transform<double, 3, Eigen::Affine> F_reg(
            Eigen::Translation<double, 3>(-100, 100, 50) *
            Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX())
    );

    //Hold whether terminating condition is satisfied
    bool term = false;

    //Maintain a counter for debugging
    int counter = 0;

    while (!term) {
        std::cout << counter << std::endl;
        cis::PointCloud A;
        cis::PointCloud B;
        cis::PointCloud E;

        for (size_t p = 0; p < q.size(); ++p) {
            //Use project_onto_surface_kd given F_reg
            cis::Point c_k = surface.root()->find_closest_point(F_reg*q.at(p));
            cis::Point d = c_k - F_reg * q.at(p);

            //If valid pair, add to the clouds above
            if (d.norm() < eta) {
                //std::cerr << d.norm() << std::endl;
                A.add_point(q.at(p));
                B.add_point(c_k);
            }
        }

        //Use Horn method to find new best transformation from A to B
        F_reg = cloud_to_cloud(A,B);

        //Determine error and update eta
        double sig = 0;
        double mean = 0;
        for (size_t i = 0; i < A.size(); i++) {
            Point e = B.at(i) - F_reg * A.at(i);
            E.add_point(e);
            sig = sig + e.squaredNorm();
            mean = mean + e.norm();
        }
        sigma.push_back(sqrt(sig)/E.size());
        mean_error.push_back(mean/E.size());
        //std::cerr << mean_error.back() << std::endl;

        eta = 3*eta; //Maybe this is the last element? May have to confirm
        //if termination condition is met, update term
        if (mean_error.size() > 2 && mean_error.back() < 5) {
            double crit = mean_error.back() / mean_error.at(mean_error.size() - 2);
            term = (gamma <= crit &&
                   crit <= 1) || mean_error.back() < 1;
            //std::cout << crit << std::endl;

        }
        counter = counter + 1;
    }

    return F_reg;
}

cis::Point cis::project_onto_surface_kdtree(const cis::Point &p,std::shared_ptr<cis::Surface::Division> root) {
    return root->find_closest_point(p);



}
