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
#include "files.h"
#include "horn.h"

namespace cis {
//Use this header file to define the work needed to determine the expected values for Ci on calibration object
    template<typename T>
    void distortion_calibration(const std::string body_file, const std::string readings_file) {
        distortion_calibration(CalibrationFile<T>(body_file), CalibrationFile<T>(readings_file));
    }

    template<typename T>
    void distortion_calibration(const CalibrationFile <T> &body, const CalibrationFile <T> &readings) {

        //Point clouds from body object
        const PointCloud<T> &d = body.opt_marker_embase();
        const PointCloud<T> &a = body.opt_marker_calobj();
        const PointCloud<T> &c = body.em_marker_calobj();



        //Now, want to estimate, F_D, F_A on every frame. Use to estimate C_i
        Eigen::Transform<T, 3, Eigen::Affine> F_A, F_D;
        for (int i = 0; i < readings.size(); i++) {
            //Estimate F_A
            const PointCloud<T> &A = readings.opt_marker_calobj(i);
            F_A = cloud_to_cloud(a, A);
            //Estimate F_D
            const PointCloud<T> &D = readings.opt_marker_embase(i);
            F_D = cloud_to_cloud(d, D);
            //Compute Theoretical C_i
            std::cout << F_A.rotation() << std::endl;
            std::cout << F_A.translation() << std::endl;

            for (int j = 0; j < c.size(); j++) {
                auto C_i = F_D.inverse() * F_A * c.at(j);
                //write these results to a file to compare to the results expected
                //Really rusty on the C++, could be absolutely batty.
                std::cout << C_i << std::endl;
            }
        }
    }
}
TEST_CASE("Sample Data Calibration Object") {
    using namespace cis;
    //For now, use make up data
    const std::string tmp_body_file("_cis_tmp_file_body");
    const std::string tmp_readings_file("_cis_tmp_file_readings");

    {
        std::ofstream t1(tmp_body_file);
        t1 << "1,2,\t3, NAME\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n5,5,5\n6,6,6";
    }

    {
        std::ofstream t2(tmp_readings_file);
        t2 << "1,2,\t3,2, NAME\n1, 5 ,6\n2,3,5\n3,2,4\n4,4,4\n5,5,5\n6,6,6\n2,7 ,8\n3,4,5\n8,7,6\n4,4,4\n5,5,5\n6,6,6";
    }

    distortion_calibration<double>(tmp_body_file, tmp_readings_file);

    remove(tmp_body_file.c_str());
    remove(tmp_readings_file.c_str());
}



#endif //CIS_CAL_DISTORTION_CALIBRATION_H
