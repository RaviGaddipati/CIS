//
// Created by Doran W on 10/17/16.
//

#ifndef CIS_CAL_PIVOT_CALIBRATION_H
#define CIS_CAL_PIVOT_CALIBRATION_H

#include "horn.h"


//Use this header file to define the operations necessary to complete the pivot calibration
namespace cis {
/**
 * @brief
 * Given multiple frames, compute the post location as well as the vector from the first frame to the tip.
 * @details
 * The system Ax=b is formed and solved for x, for frames 0..N
 * A = {{R_0,-I},...,{R_N,-I}}
 * x = {{t},{post}}^T
 * b = {{p_0},...,{p_n}}^T
 * where {R_i,p_i} = the trasnformation of the frame from the origin to the frame.
 * @param readings
 * @return x vector: {t, post}
 */
    template<typename T>
    Eigen::Matrix<T, 6, 1>
    pivot_calibration(const std::vector<PointCloud<T>> &frames) {
        if (frames.size() < 2) throw std::invalid_argument("At least 2 frames required for a complete solution.");
        // The first frame is moved to the tracker origin
        const auto reference_frame = frames.at(0).center();

        // {{R_0,-I},...,{R_N,-I}}
        Eigen::Matrix<T, Eigen::Dynamic, 6> A;
        A.resize(3 * frames.size(), Eigen::NoChange);
        Eigen::Matrix<T, Eigen::Dynamic, 1> b;
        b.resize(3 * frames.size(), Eigen::NoChange);
        const Eigen::Matrix<T, 3, 3> neg_ident = -Eigen::Matrix<T, 3, 3>::Identity();

        // Compute the transformation and build A, b matricies
        for (size_t i = 0; i < frames.size(); ++i) {
            //Need to do the reverse: Take the original reference frame (which is constant throughout all iterations) to the new points
            //const auto trans = cloud_to_cloud(frames.at(i), reference_frame);
            const auto trans = cloud_to_cloud(reference_frame, frames.at(i));
            A.block(3 * i, 0, 3, 3) = trans.rotation();
            A.block(3 * i, 3, 3, 3) = neg_ident;
            b.block(3 * i, 0, 3, 1) = -trans.translation(); //Needs to be the negative translation
        }

        // Solve the system
        return A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
    }
}

TEST_CASE ("Pivot Calibration") {
    using namespace cis;

    const int num_frames = 10;
    const int num_points = 10;

    // Vector from probe frame to probe tip
    const Eigen::Matrix<double, 3, 1> t = Eigen::Matrix<double, 3, 1>::Random();
    const Eigen::Matrix<double, 3, 1> post = {1, 2, 1};//Eigen::Matrix<double, 3, 1>::Random();

    // Random set of points, centered on origin.
    PointCloud<double> probe_cloud{{{0, 1, 2},
                                           {2, 3, 4},
                                           {3, 2, 1}}};

    std::cout << probe_cloud.centroid() << std::endl;

//    for (int j = 0; j < num_points; ++j) {
//        probe_cloud.add_point(Eigen::Matrix<double,3,1>::Random());
//    }

    //probe_cloud.center_self();

    std::vector<PointCloud<double>> frames;

    // Create frames by rotating around the post
    for (size_t i = 0; i < num_frames; ++i) {
        // Rotates the points, then moves to the post
        Eigen::Transform<double, 3, Eigen::Affine> trans(
                //From the website I posted on facebook
                Eigen::Translation<double, 3>(post) *
                Eigen::AngleAxis<double>(i * .3, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxis<double>(i * .2, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxis<double>(i * .1, Eigen::Vector3d::UnitX()) *
                Eigen::Translation<double, 3>(-post)
        );

        //std::cout << trans.matrix() << "\n--\n" << std::endl;

        frames.push_back(probe_cloud.transform(trans));
    }


    auto pred_t = pivot_calibration(frames);
    std::cout << "\nActual:\n" << t << '\n' << post << "\n\nPredicted:\n" << pred_t << std::endl;
}


#endif //CIS_CAL_PIVOT_CALIBRATION_H
