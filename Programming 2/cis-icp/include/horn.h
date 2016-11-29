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

#include <iostream>
#include "pointcloud.h"
#include "Eigen"
#include "Eigenvalues"

namespace cis {

    /**
     * @brief
     * Given two Point clouds, compute the transformation from cloud1 to cloud2 using
     * Horn's method.
     * @param cloud1 PointCloud 1
     * @param cloud2 PointCloud 2
     * @return Eigen::Transform from cloud1 to cloud2, including rotational and translational component
     */
    Eigen::Transform<double, 3, Eigen::Affine>
    cloud_to_cloud(const PointCloud &cloud1, const PointCloud &cloud2);

    /**
     * @brief
     * Given two Point clouds, compute the transformation from cloud1 to cloud2 using
     * Arun's method.
     * @param cloud1 PointCloud 1
     * @param cloud2 PointCloud 2
     * @return Eigen::Transform from cloud1 to cloud2, including rotational and translational component
     */
    Eigen::Transform<double, 3, Eigen::Affine>
    cloud_to_cloud_arun(const PointCloud &cloud1, const PointCloud &cloud2);
}

TEST_CASE ("Horn cloud-to-cloud") {
using namespace cis;
    //Generate 4 random point clouds, transform by a random transformation, confirm that the horn method
    //finds the appropriate transformation
    for (int i = 0; i < 4; i++) {
        PointCloud pc1;
        for (int j = 0; j < 5; j++) {
            Point toAdd = {double(rand() % 10), double(rand() % 10), double(rand() % 10) };
            pc1.add_point(toAdd);
        }
        {
            Eigen::Translation<double, 3> trans(1, 1, 1);
            for (int i = 0; i < pc1.size(); ++i) {
                pc1.set(i, trans * pc1.at(i));
            }
        }

        // Transform first set of points
        Eigen::Transform<double, 3, Eigen::Affine> trans(
                Eigen::Translation<double, 3>(rand() % 10, rand() % 10, rand() % 10) *
                Eigen::AngleAxis<double>(double(rand() % 3), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxis<double>(double(rand() % 3), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxis<double>(double(rand() % 3), Eigen::Vector3d::UnitX())
        );

        PointCloud pc2;
        for (size_t j = 0; j < pc1.size(); ++j) {
            pc2.add_point(trans * pc1.at(j));
        }


        auto estimated = cloud_to_cloud(pc1, pc2);
        // isApprox because floating point
                CHECK(estimated.rotation().isApprox(trans.rotation()));
                CHECK(estimated.translation().isApprox(trans.translation()));
    }
}

#endif //CIS_CAL_HORN_H
