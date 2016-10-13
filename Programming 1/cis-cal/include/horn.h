//
// Created by gaddra on 10/13/16.
//

#ifndef CIS_CAL_HORN_H
#define CIS_CAL_HORN_H

#include "pointcloud.h"
#include "Eigen"
#include "Eigenvalues"

template<typename T>
Eigen::Quaternion<T> find_rotation(const PointCloud<T> &cloud1, const PointCloud<T> &cloud2) {
    if (cloud1.size() != cloud2.size()) throw std::invalid_argument("Point clouds are different sizes.");
    const auto pc1 = cloud1.center();
    const auto pc2 = cloud2.center();
    const auto len = pc1.size();

    Eigen::Matrix<T, 3, 3> H, tmp;
    H.setZero();
    for (int i = 0; i < len; ++i) {
        // Set all cols to (ax,ay,az)
        tmp.col(0) = pc1.at(i);
        tmp.col(1) = pc1.at(i);
        tmp.col(2) = pc1.at(i);

        // Array wise multiplication to get h
        tmp.row(0).array() *= pc2.at(i).array();
        tmp.row(1).array() *= pc2.at(i).array();
        tmp.row(2).array() *= pc2.at(i).array();
        H += tmp;
    }

    // 4x4 Symmetric matrix
    Eigen::Matrix<T, 4, 4> G;
    Eigen::Matrix<T, 3, 1> delta = {H(1, 2) - H(2, 1), H(2, 0) - H(0, 2), H(0, 1) - H(1, 0)};
    G(0, 0) = H.trace();
    G.block(0, 1, 1, 3) = delta.transpose();
    G.block(1, 0, 3, 1) = delta;
    G.block(1, 1, 3, 3) = H + H.transpose();
    G.block(1, 1, 3, 3) -= H.trace() * Eigen::Matrix<T, 3, 3>::Identity();

    // Find eigenvalues/eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 4, 4>> es;
    es.compute(G);
    if (es.info() != Eigen::Success) throw std::invalid_argument("Error computing eigenvalues.");

    // eigenvector corresponding to biggest eigenvalue is quternion
    typename Eigen::Matrix<T, 4, 1>::Index max_eigen_row;
    es.eigenvalues().maxCoeff(&max_eigen_row);

    return Eigen::Quaternion<T>{es.eigenvectors().col(max_eigen_row)};
}

TEST_CASE ("Horn") {
    PointCloud<double> pc1{{{1, 2, 3}, {2, 3, 4}, {3, 4, 5}}};
    PointCloud<double> pc2{{{9, 8, 7}, {8, 7, 6}, {7, 6, 5}}};
    auto q = find_rotation(pc1, pc2);
}

#endif //CIS_CAL_HORN_H
