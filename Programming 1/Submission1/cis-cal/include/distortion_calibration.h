/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date October 13, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Performs a distortion calibration from a calibration object and readings.
 *
 * @file
 */

#ifndef CIS_CAL_DISTORTION_CALIBRATION_H
#define CIS_CAL_DISTORTION_CALIBRATION_H

#include <string>
#include <doctest.h>
#include "pointcloud.h"
#include "files.h"
#include "horn.h"

namespace cis {

    /**
     * @brief
     * Computes the expected values of the points after compensating from distortion.
     * @param body
     * @param readings
     * @return Expected cords
     */
    template<typename T>
    std::vector<PointCloud<T>> distortion_calibration(const CalBody<T> &body, const CalReadings<T> &readings) {

        //Point clouds from body object
        const PointCloud<T> &d = body.opt_marker_embase();
        const PointCloud<T> &a = body.opt_marker_calobj();
        const PointCloud<T> &c = body.em_marker_calobj();

        //Now, want to estimate, F_D, F_A on every frame. Use to estimate C_i
        Eigen::Transform<T, 3, Eigen::Affine> F_A, F_D;
        std::vector<PointCloud<T>> ret(readings.em_marker_calobj().size());
        for (int i = 0; i < readings.em_marker_calobj().size(); i++) {
            //Estimate F_A
            const PointCloud<T> &A = readings.opt_marker_calobj().at(i);
            F_A = a.transformation_to(A);
            //Estimate F_D
            const PointCloud<T> &D = readings.opt_marker_embase().at(i);
            F_D = d.transformation_to(D);
            //Compute Theoretical C_i
            ret[i] = c.transform(F_D.inverse() * F_A);
        }
        return ret;
    }
}


#endif //CIS_CAL_DISTORTION_CALIBRATION_H
