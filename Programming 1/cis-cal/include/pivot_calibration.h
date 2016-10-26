/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date October 23, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Performs a pivot calibration.
 *
 * @file
 */

#ifndef CIS_CAL_PIVOT_CALIBRATION_H
#define CIS_CAL_PIVOT_CALIBRATION_H

#include "horn.h"
#include "bernstein.h"
#include <doctest.h>

namespace cis {
    /**
     * @brief
     * Given multiple frames, compute the post location as well as the vector from the first frame to the tip.
     * @details
     * The system Ax=b is formed and solved for x, for frames 0..N
     * A = {{R_0,-I},...,{R_N,-I}}
     * x = {{t},{post}}^T
     * b = {{-p_0},...,{-p_n}}^T
     * where {R_i,p_i} = the trasnformation of the frame from the origin to the frame.
     * @param frames
     * @return x vector: {t, post}
     */
    Eigen::Matrix<double, 6, 1>
    pivot_calibration(const std::vector<PointCloud> &frames);


    Eigen::Matrix<double,6,1>
    pivot_calibration(const std::vector<PointCloud> &frames,
                      const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> &fn);

     /**
     * @brief
     * Given multiple frames, compute the post location as well as the vector from the first frame to the tip.
      * Uses 2 frame vectors, where the optical tracker position can change relative to the EM tracker.
     * @details
     * The system Ax=b is formed and solved for x, for frames 0..N
     * A = {{R_OH^N,-R_OD^N},...,{R_OH^N,-R_OD^N}}
     * x = {{t},{post}}^T
     * b = {{p_OD_n - p_OH_n},...,{p_OD_n - p_OH_n}}^T
     * where {R_i,p_i} = the trasnformation of the frame from the origin to the frame.
     * @param opt Optical tracker frames
     * @param em EM tracker frames
     * @return x vector: {t, post}
     */
    Eigen::Matrix<double, 6, 1>
    pivot_calibration_opt(const std::vector<PointCloud> &opt, const std::vector<PointCloud> &em);
}

TEST_CASE ("Pivot Calibration") {
    using namespace cis;

    const size_t num_frames = 10;

    // Vector to the post in space
    const Point post = {1, 1, 1};//Eigen::Matrix<double, 3, 1>::Random();

    // Random set of points, centered on origin.
    PointCloud probe_cloud{{{0, 1, 2},
                                           {2, 3, 4},
                                           {3, 2, 1}}};

    //The "tip" is the vector from the centroid of the point cloud to the post
    const Point t = post - probe_cloud.centroid();

    std::vector<PointCloud> frames;

    // Create frames by rotating around the post
    for (size_t i = 0; i < num_frames; ++i) {
        Eigen::Transform<double, 3, Eigen::Affine> trans(
                // rotate, move back to post
                Eigen::Translation<double, 3>(post) *
                Eigen::AngleAxis<double>(i * .3, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxis<double>(i * .2, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxis<double>(i * .1, Eigen::Vector3d::UnitX()) *
                Eigen::Translation<double, 3>(-post)
        );
        frames.push_back(probe_cloud.transform(trans));
    }

    auto pred_t = pivot_calibration(frames);

    CHECK(t.isApprox(pred_t.block(0,0,3,1)));
    CHECK(post.isApprox(pred_t.block(3,0,3,1)));
}


#endif //CIS_CAL_PIVOT_CALIBRATION_H
