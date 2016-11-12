/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date October 23, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Compute a registration from one reference to another
 * given fiducials and a probe tip.
 *
 * @file
 */
#ifndef CIS_CAL_REGISTRATION_H
#define CIS_CAL_REGISTRATION_H

#include "pointcloud.h"

namespace cis {

    /**
     * @brief
     * Computes the registration between CT coords and EM coords.
     * @param em_fiducuals EM coords of the probe cloud, corrected
     * @param ct_fiducials
     * @param fn Interpolation function
     * @param scale_min scaling used for FN
     * @param scale_max Scaling used for Fn
     * @param ptip tip computed from pivot calibration
     * @return Freg
     */
    Eigen::Transform<double, 3, Eigen::Affine>
    register_frames(const cis::PointCloud &reference_frame,
                    const std::vector<cis::PointCloud> &em_fiducuals,
                    const cis::PointCloud &ct_fiducials,
                    const Eigen::MatrixXd &fn,
                    const cis::Point &scale_min, const cis::Point &scale_max,
                    const Point &ptip);

    /**
     * @brief
     * Given a registration, compute the CT coordiantes of an EM cloud.
     * @param frames
     * @param fn
     * @param scale_min
     * @param scale_max
     * @param ptip
     * @param Freg
     * @return Transformed EM probe tips.
     */
    PointCloud em_to_ct(const cis::PointCloud &reference_frame, const std::vector<cis::PointCloud> &frames, const Eigen::MatrixXd &fn,
                        const cis::Point &scale_min, const cis::Point &scale_max, const Point &ptip,
                        const Eigen::Transform<double, 3, Eigen::Affine> &Freg);

}

#endif //CIS_CAL_REGISTRATION_H
