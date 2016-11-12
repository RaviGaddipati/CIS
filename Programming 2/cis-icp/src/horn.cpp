/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date October 23, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Computes a rigid body to rigid body transformation
 *
 * @file
 */

#include "horn.h"

Eigen::Transform<double, 3, Eigen::Affine>
cis::cloud_to_cloud(const cis::PointCloud &cloud1, const cis::PointCloud &cloud2) {
    if (cloud1.size() != cloud2.size()) throw std::invalid_argument("Point clouds are different sizes.");
    // Center both clouds on origin
    const auto len = cloud1.size();
    const auto c1_centroid = cloud1.centroid();
    const auto c2_centroid = cloud2.centroid();

    Eigen::Matrix<double, 3, 3>
            H = Eigen::Matrix<double, 3, 3>::Zero(), // Sum of all (x,y,z) pairs
            t1 = Eigen::Matrix<double, 3, 3>::Zero(), // Temporary for matrix multiplication
            t2 = Eigen::Matrix<double, 3, 3>::Zero();

    // Build H-matrix
    for (int i = 0; i < len; ++i) {
        t1.col(0) = cloud1.at(i) - c1_centroid;
        t2.row(0) = cloud2.at(i) - c2_centroid;
        H += t1 * t2;
    }

    // 4x4 Symmetric matrix
    Eigen::Matrix<double, 4, 4> G;
    Eigen::Matrix<double, 3, 1> delta = {H(1, 2) - H(2, 1),
                                    H(2, 0) - H(0, 2),
                                    H(0, 1) - H(1, 0)};
    G(0, 0) = H.trace();
    G.block(0, 1, 1, 3) = delta.transpose();
    G.block(1, 0, 3, 1) = delta;
    G.block(1, 1, 3, 3) = H + H.transpose();
    G.block(1, 1, 3, 3) -= H.trace() * Eigen::Matrix<double, 3, 3>::Identity();

    // Find eigenvalues/eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 4, 4>> es;
    es.compute(G);

    if (es.info() != Eigen::Success) {
        throw std::runtime_error("Error computing eigenvalues: " + std::to_string(es.info()));
    }

    // eigenvector corresponding to biggest eigenvalue is quaternion
    typename Eigen::Matrix<double, 4, 1>::Index max_eigen_row;
    es.eigenvalues().maxCoeff(&max_eigen_row);
    const auto &eig_vec = es.eigenvectors().col(max_eigen_row);
    // Can't pass vec directly since Quaternion interprets as quat as [x,y,z,w]
    Eigen::Quaternion<double> rot{eig_vec(0), eig_vec(1), eig_vec(2), eig_vec(3)};

    Eigen::Translation<double, 3> trans(c2_centroid - (rot * c1_centroid));

    return Eigen::Transform<double, 3, Eigen::Affine>(trans * rot); // Applies rotation first
}


Eigen::Transform<double, 3, Eigen::Affine>
cis::cloud_to_cloud_arun(const cis::PointCloud &cloud1, const cis::PointCloud &cloud2) {
    const auto pc1 = cloud1.center();
    const auto pc2 = cloud2.center();
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero(), X;
    for (int i = 0; i < pc1.size(); ++i) {
        H = H + pc1.at(i) * (pc2.at(i).transpose());
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    X = V * (U.transpose());

    const double det = X.determinant();
    if ((int) det != 1) {
        auto V = svd.matrixV();
        Eigen::Vector3d v = V.block<3,1>(0,2);
        V.block<3,1>(0,2) = v * -1;
        X = V * U.transpose();
    }

    Eigen::Transform<double, 3, Eigen::Affine> ret;
    ret.matrix().block<3,3>(0,0) = X;
    ret.matrix().block<3,1>(0,3) = cloud2.centroid() - X * cloud1.centroid();
    ret.matrix().block<1,4>(3,0) = Eigen::Matrix<double,1,4>{0,0,0,1};
    return ret;
}
