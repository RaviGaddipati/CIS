//
// Created by gaddra on 10/26/16.
//

#include "registration.h"
#include "distortion_calibration.h"

Eigen::Transform<double, 3, Eigen::Affine>
cis::register_frames(const std::vector<cis::PointCloud> &em_fiducuals,
                const cis::PointCloud &ct_fiducials,
                const Eigen::MatrixXd &fn,
                const cis::Point &scale_min, const cis::Point &scale_max,
                const Point &ptip) {
    const std::vector<cis::PointCloud> calibrated = cis::correct_frames(em_fiducuals, fn, scale_min, scale_max);
    const cis::PointCloud reference_frame = calibrated.at(0).center();

    cis::PointCloud v;
    for (const auto &frame : calibrated) {
        v.add_point(cis::cloud_to_cloud(reference_frame, frame) * ptip);
    }

    const auto freg = cis::cloud_to_cloud(v, ct_fiducials);
    return freg;
}

cis::PointCloud cis::em_to_ct(const std::vector<cis::PointCloud> &frames, const Eigen::MatrixXd &fn,
                         const cis::Point &scale_min, const cis::Point &scale_max, const cis::Point &ptip,
                         const Eigen::Transform<double, 3, Eigen::Affine> &Freg) {
    const std::vector<cis::PointCloud> calibrated = cis::correct_frames(frames, fn, scale_min, scale_max);
    const cis::PointCloud reference_frame = calibrated.at(0).center();

    cis::PointCloud v;
    for (const auto &frame : calibrated) {
        v.add_point(Freg * (cis::cloud_to_cloud(reference_frame, frame) * ptip));
    }
    return v;
}