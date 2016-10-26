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

    using Point = PointCloud<double>::Point;

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
        std::vector<PointCloud<T>> ret(readings.em_marker_calobj().size()); //Expected values
        for (int i = 0; i < readings.em_marker_calobj().size(); i++) {
            //Estimate F_A
            const PointCloud<T> &A = readings.opt_marker_calobj().at(i);
            F_A = cloud_to_cloud(a, A);
            //Estimate F_D
            const PointCloud<T> &D = readings.opt_marker_embase().at(i);
            F_D = cloud_to_cloud(d, D);
            //Compute Theoretical C_i
            ret[i] = c.transform(F_D.inverse() * F_A);
        }
        return ret;
    }

    /**
     * @brief
     * Computes the interpolation function given the sensor values and the expected values.
     * Dynamic matrix is used to prevent stack overflow.
     * @param readings
     * @tparam DEGREE Degree of the polynomial to compute.
     */
    template<typename T, size_t DEGREE>
    Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>
    distortion_function(const CalReadings<T> &readings, const std::vector<PointCloud<T>> &expected) {
        //Now, want to iterate over each frame of data, adding expected points to
        //P_mat, and computing the Bernstein polynomial representation of the measured
        const size_t num_frames = readings.em_marker_calobj().size();
        const size_t num_points = readings.em_marker_calobj().at(0).size();
        const size_t num_rows = num_frames * num_points;

        //Store all expected points in x,y,z form
        Eigen::Matrix<T, Eigen::Dynamic, 3>
                P_mat = Eigen::Matrix<T, Eigen::Dynamic, 3>::Zero(num_rows, 3);
        //Store all Bernstein representations
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>
                F_mat = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_rows, cexp_pow(DEGREE + 1, 3));
        size_t counter = 0;
        Point u;

        for (size_t i = 0; i < num_frames; i++) {
            const PointCloud<T> &C = readings.em_marker_calobj().at(i);
            const PointCloud<T> &C_exp = expected.at(i);
            for (size_t j = 0; j < C.size(); j++) {
                P_mat.row(counter) = C_exp.at(j);
                u = scale_to_box(C.at(j), min(readings.em_marker_calobj()),
                                       max(readings.em_marker_calobj()));
                F_mat.row(counter) = interpolation_poly<DEGREE>(u);
                ++counter;
            }
        }

        //Now we have all of the frames and can estimate the weights
        return F_mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(P_mat);
    }
}

TEST_CASE("Distortion Function") {

}

#endif //CIS_CAL_DISTORTION_CALIBRATION_H
