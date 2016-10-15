/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date October 15, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Computes the transformation from one point cloud to another.
 *
 * @file
 */

#ifndef CIS_CAL_HORN_H
#define CIS_CAL_HORN_H

#include "pointcloud.h"
#include "Eigen"
#include "Eigenvalues"

/**
 * @brief
 * Given two Point clouds, compute the transformation from cloud1 to cloud2 using
 * Horn's method.
 * @param cloud1 PointCloud
 * @param cloud2 PointCloud
 * @return Eigen::Transform, including rotational and translational component
 */
template<typename T>
Eigen::Transform<T, 3, Eigen::Affine> cloud_to_cloud(const PointCloud<T> &cloud1, const PointCloud<T> &cloud2) {
    if (cloud1.size() != cloud2.size()) throw std::invalid_argument("Point clouds are different sizes.");
    // Center both clouds on origin
    const auto pc1 = cloud1.center();
    const auto pc2 = cloud2.center();
    const auto len = pc1.size();

    Eigen::Matrix<T, 3, 3>
            H = Eigen::Matrix<T, 3, 3>::Zero(), // Sum of all (x,y,z) pairs
            t1 = Eigen::Matrix<T, 3, 3>::Zero(), // Temporary for matrix multiplication
            t2 = Eigen::Matrix<T, 3, 3>::Zero();

    for (int i = 0; i < len; ++i) {
        t1.col(0) = pc1.at(i);
        t2.row(0) = pc2.at(i);
        H += t1 * t2;
    }

    // 4x4 Symmetric matrix
    Eigen::Matrix<T, 4, 4> G;
    Eigen::Matrix<T, 3, 1> delta = {H(1, 2) - H(2, 1),
                                    H(2, 0) - H(0, 2),
                                    H(0, 1) - H(1, 0)};
    G(0, 0) = H.trace();
    G.block(0, 1, 1, 3) = delta.transpose();
    G.block(1, 0, 3, 1) = delta;
    G.block(1, 1, 3, 3) = H + H.transpose();
    G.block(1, 1, 3, 3) -= H.trace() * Eigen::Matrix<T, 3, 3>::Identity();

    // Find eigenvalues/eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 4, 4>> es;
    es.compute(G);
    if (es.info() != Eigen::Success) throw std::runtime_error("Error computing eigenvalues.");

    // eigenvector corresponding to biggest eigenvalue is quaternion
    typename Eigen::Matrix<T, 4, 1>::Index max_eigen_row;
    es.eigenvalues().maxCoeff(&max_eigen_row);
    const auto &eig_vec = es.eigenvectors().col(max_eigen_row);
    // Can't pass vec directly since Quaternion interprets as quat as [x,y,z,w]
    Eigen::Quaternion<T> rot{eig_vec(0), eig_vec(1), eig_vec(2), eig_vec(3)};

    Eigen::Translation<T, 3> trans(cloud2.centroid() - (rot * cloud1.centroid()));
    return Eigen::Transform<T, 3, Eigen::Affine>(trans * rot); // Applies rotation first
}

TEST_CASE ("Horn cloud-to-cloud") {
    // Group of points translated to (1,1,1)
    PointCloud<double> pc1{{{1, 2, 3}, {0, 1, .5}, {0, 0, 1}, {-1, 0, 0}, {3, 8, 7}}};
    {
        Eigen::Translation<double, 3> trans(1, 1, 1);
        for (int i = 0; i < pc1.size(); ++i) {
            pc1.set(i, trans * pc1.at(i));
        }
    }

    // Transform first set of points
    Eigen::Transform<double, 3, Eigen::Affine> trans(
            Eigen::Translation<double, 3>(1, 2, 3) *
            Eigen::AngleAxis<double>(3, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxis<double>(2, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX())
    );

    PointCloud<double> pc2;
    for (size_t i = 0; i < pc1.size(); ++i) {
        pc2.add_point(trans * pc1.at(i));
    }

    // Calculate transformation
    auto estimated = cloud_to_cloud(pc1, pc2);

    // isApprox because floating point
            CHECK(estimated.rotation().isApprox(trans.rotation()));
            CHECK(estimated.translation().isApprox(trans.translation()));

}

#endif //CIS_CAL_HORN_H
