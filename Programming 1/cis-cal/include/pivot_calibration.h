//
// Created by Doran W on 10/17/16.
//

#ifndef CIS_CAL_PIVOT_CALIBRATION_H
#define CIS_CAL_PIVOT_CALIBRATION_H


//Use this header file to define the operations necessary to complete the pivot calibration

template<T>
void pivot_calibration(const std::string &em_pivot_file, const std::sting &opt_pivot_file) {

    //Read the data from each pivot calibration file
    //NOTE: The existing files.h is currently not structured to accept these general data files and
    //needs to be changed

    //Same form as currently used in files.h, but just with the appropriate transformations
    /*
    CalibrationFile<T> emReadings; //em_marker_pointer
    CalibrationFile<T> optReadings; //opt_marker_pointer, opt_marker_embase
    */

    //For each transformation F_OD, F_OH, and F_MG (optical-embase, optical-pointer, em-pointer), estimate the transformation
    //Using the data present. Example here is for H (data collected from the optical pointer)

    //First, compute the representation of the vectors in the H frame by taking the difference of every vector from
    //centroid

    /*
    PointCloud h;
    Point h_hat = centroid of PointCloud H in optReadings (H.centroid()) on the first frame
     for each vector in PointCloud H for each frame of data
        Add H_i - h_hat to PointCloud h
     */

    //You now have a two entire pointclouds and can now run horn to generate the rotation and translation between the two

    /*
    for each frame within optReadings
     Compute cloud_to_cloud(h,H), store this transformation somewhere
     Compute cloud_to_cloud(d,D)
     Compute cloud_to_cloud(g,G)
     Because our horn method finds the transformation from the first argument to the second, we have to use the order above to
     get the correct transformation.
     */

    //Using these transformations, we can compute the pivot calibration and find the distance to the tips of the pointers
    //Not sure how to best write this, but the math that I completed on Overleaf is dependent on rotations and translations
    //computed above. Just need to make the matrices and make the product.

}


#endif //CIS_CAL_PIVOT_CALIBRATION_H
