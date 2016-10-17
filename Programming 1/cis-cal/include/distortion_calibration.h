//
// Created by Doran W on 10/17/16.
//

#ifndef CIS_CAL_DISTORTION_CALIBRATION_H
#define CIS_CAL_DISTORTION_CALIBRATION_H

#include <string>
#include <doctest.h>
#include "files.h"
#include "horn.h"
//Use this header file to define the work needed to determine the expected values for Ci on calibration object

void distortion_calibration(std::string file_body, std::string file_readings) {
    //Read in the calibration body details and calibration readings using files.h
    CalibrationFile<double> body = CalibrationFile<double>(file_body);
    CalibrationFile<double> readings = CalibrationFile<double>(file_readings);

    //Point clouds from body object
    auto d = body.frame().opt_marker_embase;
    auto a = body.frame().opt_marker_calobj;
    auto c = body.frame().em_marker_calobj;



    //Now, want to estimate, F_D, F_A on every frame. Use to estimate C_i
    for(int i = 0; i < readings.size(); i++) {
        //Estimate F_A
        auto A = readings.frame(i).opt_marker_calobj;
        auto F_A = cloud_to_cloud(a,A);
        //Estimate F_D
        auto D = readings.frame(i).opt_marker_embase;
        auto F_D = cloud_to_cloud(d,D);
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

TEST_CASE("Sample Data Calibration Object") {
    //For now, use make up data
    const std::string tmp_body_file("_cis_tmp_file_body");
    std::ofstream t1(tmp_body_file);
    t1 << "1,2,\t3, NAME\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n5,5,5\n6,6,6";
    t1.close();

    const std::string tmp_readings_file("_cis_tmp_file_readings");
    std::ofstream t2(tmp_readings_file);
    t2 << "1,2,\t3,2, NAME\n1, 5 ,6\n2,3,5\n3,2,4\n4,4,4\n5,5,5\n6,6,6\n2,7 ,8\n3,4,5\n8,7,6\n4,4,4\n5,5,5\n6,6,6";
    t2.close();

    distortion_calibration(tmp_body_file,tmp_readings_file);
}



#endif //CIS_CAL_DISTORTION_CALIBRATION_H
