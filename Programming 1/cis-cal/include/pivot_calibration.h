//
// Created by Doran W on 10/17/16.
//

#ifndef CIS_CAL_PIVOT_CALIBRATION_H
#define CIS_CAL_PIVOT_CALIBRATION_H

#include "horn.h"


//Use this header file to define the operations necessary to complete the pivot calibration
namespace cis {
/**
 * Given multiple frames and the location of a post,
 * return the vector from the first frame to the tip.
 * @param readings
 * @return
 */
    template<typename T>
    Eigen::Matrix<T, 3, 1>
    pivot_calibration(const std::vector<PointCloud<T>> &frames,
                      const Eigen::Matrix<T, 3, 1> &post_position) {
        if (frames.size() < 2) throw std::invalid_argument("Multiple frames needed for pivot calibration.");
        const auto reference_origin = frames.at(0).centroid();

        std::vector<Eigen::Transform<T, 3, Eigen::Affine>> frame_transforms;
        for (const auto &frame : frames) {
            const auto g_j = frame - reference_origin;
            frame_transforms.push_back(cloud_to_cloud(g_j, frames.at(0)));
            //    std::cout << frame_transforms.back().matrix() * post_position.homogeneous() << std::endl << std::endl;
        }

        const auto &sol = frame_transforms.at(2).matrix().jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(
                post_position.homogeneous());
        return {sol(0), sol(1), sol(2)};
        //Read the data from each pivot calibration file
        //NOTE: The existing files.h is currently not structured to accept these general data files and
        //needs to be changed

        //Same form as currently used in files.h, but just with the appropriate transformations
        /*
        CalibrationFile<T> emReadings; //em_marker_pointer
        CalibrationFile<T> optReadings; //opt_marker_pointer, opt_marker_embase
        */

        //For each transformation F_OD, F_OH, and F_MG (optical-embase, optical-pointer, em-pointer), estimate the transformation
        //Using the data present. Example here is for H (data collected from the optical pointer)

        //First, compute the representation of the vectors in the H frame by taking the difference of every vector from
        //centroid

        /*
        PointCloud h;
        Point h_hat = centroid of PointCloud H in optReadings (H.centroid()) on the first frame
         for each vector in PointCloud H for each frame of data
            Add H_i - h_hat to PointCloud h
         */

        //You now have a two entire pointclouds and can now run horn to generate the rotation and translation between the two

        /*
        for each frame within optReadings
         Compute cloud_to_cloud(h,H), store this transformation somewhere
         Compute cloud_to_cloud(d,D)
         Compute cloud_to_cloud(g,G)
         Because our horn method finds the transformation from the first argument to the second, we have to use the order above to
         get the correct transformation.
         */

        //Using these transformations, we can compute the pivot calibration and find the distance to the tips of the pointers
        //Not sure how to best write this, but the math that I completed on Overleaf is dependent on rotations and translations
        //computed above. Just need to make the matrices and make the product.

    }
}

TEST_CASE ("Pivot") {
    using namespace cis;

    // Vector from probe frame to probe tip
    const Eigen::Matrix<double, 3, 1> t = Eigen::Matrix<double, 3, 1>::Random();
    const Eigen::Matrix<double, 3, 1> post = {0, 0, 0};

    // Random set of points, centered on the post.
    PointCloud<double> probe_cloud{{{0, 1, 2},
                                           {2, 3, 4},
                                           {3, 2, 1}}};
    probe_cloud.center_self() -= t;

    std::vector<PointCloud<double>> frames;
    // Create 10 frames by rotating around the post
    for (size_t i = 0; i < 10; ++i) {
        Eigen::Transform<double, 3, Eigen::Affine> trans(
                Eigen::Translation<double, 3>(t) *
                Eigen::Translation<double, 3>(post) *
                Eigen::AngleAxis<double>(i * .3, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxis<double>(i * .2, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxis<double>(i * .1, Eigen::Vector3d::UnitX())
        );
        frames.push_back(probe_cloud.transform(trans));
    }


    auto pred_t = pivot_calibration(frames, post);
    std::cout << "\nt:\n" << t << "\n\nPredicted:\n" << pred_t << std::endl;
}


#endif //CIS_CAL_PIVOT_CALIBRATION_H
