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
    template<typename T>
    Eigen::Matrix<T, 6, 1>
    pivot_calibration(const std::vector<PointCloud<T>> &frames) {
        // The first frame is moved to the origin
        const auto reference_frame = frames.at(0).center();

        // {{R_0,-I},...,{R_N,-I}}
        Eigen::Matrix<T, Eigen::Dynamic, 6> A;
        A.resize(3 * frames.size(), Eigen::NoChange);
        Eigen::Matrix<T, Eigen::Dynamic, 1> b;
        b.resize(3 * frames.size(), Eigen::NoChange);
        const Eigen::Matrix<T, 3, 3> neg_ident = -Eigen::Matrix<T, 3, 3>::Identity();

        // Compute the transformation and build A, b matricies
        for (size_t i = 0; i < frames.size(); ++i) {
            const auto trans = cloud_to_cloud(reference_frame, frames.at(i));
            A.block(3 * i, 0, 3, 3) = trans.rotation().matrix();
            A.block(3 * i, 3, 3, 3) = neg_ident;
            b.block(3 * i, 0, 3, 1) = -trans.translation().matrix(); //Needs to be the negative translation
        }

        // Solve the system
        return A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
    }


    template<typename T>
    Eigen::Matrix<T,6,1>
    pivot_calibration(const std::vector<PointCloud<T>> &frames,
                      const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &fn) {

        const typename PointCloud<T>::Point
                _min = min(frames),
                _max = max(frames);

        std::vector<cis::PointCloud<T>> calibrated(0);
        // Calibrate all the points
        Eigen::Matrix<T,3,1> cal_pt;
        for (const auto &frame : frames) {
            calibrated.emplace_back();
            auto &back = calibrated.back();
            for (size_t k = 0; k < frame.size(); ++k) {
                cal_pt = scale_to_box(frame.at(k), _min, _max);
                cal_pt = interpolation_poly<5>(cal_pt) * fn;
                back.add_point(cal_pt);
            }
        }

        return pivot_calibration(calibrated);
    }

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
    template<typename T>
    Eigen::Matrix<T, 6, 1>
    pivot_calibration_opt(const std::vector<PointCloud<T>> &opt, const std::vector<PointCloud<T>> &em) {
         if (opt.size() != em.size()) throw std::invalid_argument("Both vectors must be the same size.");
        // The first frame is moved to the origin
        const PointCloud<T> em_reference_frame = em.at(0).center();
         std::vector<PointCloud<T>> trans_opt(opt.size());
         Eigen::Matrix<T, 4, 4> transmat;
         for (int j = 0; j < opt.size(); ++j) {
             const auto trans = cloud_to_cloud(em_reference_frame, em.at(j));
             transmat.block(0,0,3,3) = trans.rotation().matrix().inverse();
             transmat.block(0,3,3,1) = -trans.rotation().matrix().inverse() * trans.translation();
             transmat.block(3,0,1,4) = Eigen::Matrix<double,1,4>{0,0,0,1};
             PointCloud<T> n;
             for (int i = 0; i < opt.at(j).size(); ++i) {
                 n.add_point((transmat * opt.at(j).at(i).homogeneous()).block(0,0,3,1));
             }
             trans_opt[j] = n;
         }
         return pivot_calibration(trans_opt);
    }
}

TEST_CASE ("Pivot Calibration") {
    using namespace cis;

    const size_t num_frames = 10;

    // Vector to the post in space
    const Eigen::Matrix<double,3,1> post = {1, 1, 1};//Eigen::Matrix<double, 3, 1>::Random();

    // Random set of points, centered on origin.
    PointCloud<double> probe_cloud{{{0, 1, 2},
                                           {2, 3, 4},
                                           {3, 2, 1}}};

    //The "tip" is the vector from the centroid of the point cloud to the post
    const Eigen::Matrix<double,3,1> t = post - probe_cloud.centroid();

    std::vector<PointCloud<double>> frames;

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
