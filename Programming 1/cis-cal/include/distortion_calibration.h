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
            F_A = a.transformation_to(A);
            //Estimate F_D
            const PointCloud<T> &D = readings.opt_marker_embase().at(i);
            F_D = d.transformation_to(D);
            //Compute Theoretical C_i
            ret[i] = c.transform(F_D.inverse() * F_A);
        }
        return ret;
    }

    template<typename T>
    Eigen::Matrix<T,125,3> distortion_function(const CalReadings<T> &readings, const std::vector<PointCloud<T>> &ret) {
        //Now, want to iterate over each frame of data, adding expected points to
        //P_mat, and computing the Bernstein polynomial representation of the measured
        const size_t num_frames = readings.em_marker_calobj().size();
        const size_t num_points = readings.em_marker_calobj().at(0).size();
        const size_t num_rows = num_frames * num_points;

        //Store all expected points in x,y,z form
        Eigen::Matrix<T, Eigen::Dynamic, 3> P_mat = Eigen::Matrix<T, Eigen::Dynamic, 3>::Zero(num_rows, 3);
        //Store all Bernstein representations
        Eigen::Matrix<T, Eigen::Dynamic, 125> F_mat = Eigen::Matrix<T, Eigen::Dynamic, 3>::Zero(num_rows, 3);
        int counter = 0;
        for (size_t i = 0; i < num_frames; i++) {
            const PointCloud<T> &C = readings.em_marker_calobj().at(i);
            const PointCloud<T> &C_exp = ret.at(i);
            for (size_t j = 0; j < C.size(); j++) {
                P_mat.row(counter) = C_exp.at(j);
                Point u = scaleToBox(C.at(j), {-500, -500, -500}, {500, 500, 500}); //NEED TO FIND q_min and q_max
                F_mat.row(counter) = computeF(u);
                ++counter;
            }

        }

        //Now we have all of the frames and can estimate the weights
        Eigen::Matrix<double, 125, 3> c_weights = F_mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(
                P_mat);
        return c_weights;

    }
}


#endif //CIS_CAL_DISTORTION_CALIBRATION_H
