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
#include "bernstein.h"
#include "Eigen"
#include "Eigenvalues"

namespace cis {


    /**
     * @brief
     * Computes the expected values of the points after compensating from distortion.
     * @param body
     * @param readings
     * @return Expected cords
     */
    std::vector<PointCloud> distortion_calibration(const CalBody &body, const CalReadings &readings);

    /**
     * @brief
     * Computes the interpolation function given the sensor values and the expected values.
     * Dynamic matrix is used to prevent stack overflow.
     * @param readings
     * @tparam DEGREE Degree of the polynomial to compute.
     */
    template<size_t DEGREE>
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>
    distortion_function(const CalReadings &readings, const std::vector<PointCloud> &expected,
                        const Point &scale_min, const Point &scale_max) {
        //Now, want to iterate over each frame of data, adding expected points to
        //P_mat, and computing the Bernstein polynomial representation of the measured
        const size_t num_frames = readings.em_marker_calobj().size();
        const size_t num_points = readings.em_marker_calobj().at(0).size();
        const size_t num_rows = num_frames * num_points;

        //Store all expected points in x,y,z form
        Eigen::Matrix<double, Eigen::Dynamic, 3>
                P_mat = Eigen::Matrix<double, Eigen::Dynamic, 3>::Zero(num_rows, 3);
        //Store all Bernstein representations
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
                F_mat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_rows, cexp_pow(DEGREE + 1, 3));
        size_t counter = 0;
        Point u;

        for (size_t i = 0; i < num_frames; i++) {
            const PointCloud &C = readings.em_marker_calobj().at(i);
            const PointCloud &C_exp = expected.at(i);
            for (size_t j = 0; j < C.size(); j++) {
                P_mat.row(counter) = C_exp.at(j);
                u = scale_to_box(C.at(j), scale_min, scale_max);
                F_mat.row(counter) = interpolation_poly<DEGREE>(u);
                ++counter;
            }
        }

        //Now we have all of the frames and can estimate the weights
        return F_mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(P_mat);
    }

    /**
     * @brief
     * Applies the interpolation function to the points
     * @param frame
     * @param fn
     * @param scale_min
     * @param scale_max
     * @return corrected points
     */
    cis::PointCloud correct_points(const cis::PointCloud &frame, const Eigen::MatrixXd &fn,
                                   const cis::Point &scale_min, const cis::Point &scale_max);

    /**
     * @brief
     * Applies the interpolation function to the points of the frames
     * @param frame
     * @param fn
     * @param scale_min
     * @param scale_max
     * @return corrected frames
     */
    std::vector<cis::PointCloud> correct_frames(const std::vector<cis::PointCloud> &frames, const Eigen::MatrixXd &fn,
                                                const cis::Point &scale_min, const cis::Point &scale_max);
}


#endif //CIS_CAL_DISTORTION_CALIBRATION_H
