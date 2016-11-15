//
// Created by Doran W on 11/14/16.
//


#include "registration.h"
#include "horn.h"

cis::PointCloud
cis::pointer_to_fixed(const cis::RigidBody &pointerRef, const cis::RigidBody &fixedRef,
                                       const std::vector <cis::PointCloud> &pointerframes,
                                       const std::vector <cis::PointCloud> &fixedframes) {
    cis::PointCloud d;
    // Must have the same number of frames
    if (pointerframes.size() != fixedframes.size()) {
        throw;
    } else {
        //Get the point cloud from the pointer referecne frame F_A and find the transform
        for (int i = 0; i < pointerframes.size() ; ++i) {
            Point temp = cloud_to_cloud(pointerRef.markers(), pointerframes.at(i)) * pointerRef.tip();
            d.add_point(cloud_to_cloud(fixedRef.markers(),fixedframes.at(i)).inverse() * temp);
        }
    }
    return d;
}
