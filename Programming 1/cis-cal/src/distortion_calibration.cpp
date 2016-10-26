//
// Created by gaddra on 10/26/16.
//

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