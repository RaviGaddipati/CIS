/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date October 23, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Compute the expected values given a calibration body.
 *
 * @file
 */

#include "distortion_calibration.h"

std::vector<cis::PointCloud> cis::distortion_calibration(const cis::CalBody &body, const cis::CalReadings &readings) {
    //Point clouds from body object
    const PointCloud &d = body.opt_marker_embase();
    const PointCloud &a = body.opt_marker_calobj();
    const PointCloud &c = body.em_marker_calobj();

    //Now, want to estimate, F_D, F_A on every frame. Use to estimate C_i
    Eigen::Transform<double, 3, Eigen::Affine> F_A, F_D;
    std::vector<PointCloud> ret(readings.em_marker_calobj().size()); //Expected values
    for (size_t i = 0; i < readings.em_marker_calobj().size(); i++) {
        //Estimate F_A
        const PointCloud &A = readings.opt_marker_calobj().at(i);
        F_A = cloud_to_cloud(a, A);
        //Estimate F_D
        const PointCloud &D = readings.opt_marker_embase().at(i);
        F_D = cloud_to_cloud(d, D);
        //Compute Theoretical C_i
        ret[i] = c.transform(F_D.inverse() * F_A);
    }
    return ret;
}


cis::PointCloud cis::correct_points(const cis::PointCloud &frame, const Eigen::MatrixXd &fn,
                               const cis::Point &scale_min, const cis::Point &scale_max) {
    cis::PointCloud calibrated;
    cis::Point cal_pt;
    for (size_t k = 0; k < frame.size(); ++k) {
        cal_pt = cis::scale_to_box(frame.at(k), scale_min, scale_max);
        cal_pt = cis::interpolation_poly<5>(cal_pt) * fn;
        calibrated.add_point(cal_pt);
    }
    return calibrated;
}

std::vector<cis::PointCloud> cis::correct_frames(const std::vector<cis::PointCloud> &frames, const Eigen::MatrixXd &fn,
                                            const cis::Point &scale_min, const cis::Point &scale_max) {
    std::vector<cis::PointCloud> corrected;
    for (const auto &frame : frames) {
        corrected.push_back(cis::correct_points(frame, fn, scale_min, scale_max));
    }
    return corrected;
}
