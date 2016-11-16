/**
 * @author Doran Walsten, Ravi Gaddipati
 * @date Nov 15, 2016
 *
 * @brief
 * Registration between fixed and mobile bodies.
 *
 * @file
 */


#include "registration.h"
#include "horn.h"

cis::PointCloud
cis::pointer_to_fixed(const cis::RigidBody &pointerRef,
                      const cis::RigidBody &fixedRef,
                      const std::vector<cis::PointCloud> &pointerframes,
                      const std::vector<cis::PointCloud> &fixedframes) {
    cis::PointCloud d;
    // Must have the same number of frames
    if (pointerframes.size() != fixedframes.size())
        throw std::invalid_argument("Different number of frames in fixed and pointer bodies.");

    //Get the point cloud from the pointer referecne frame F_A and find the transform
    std::vector<Point> temp(pointerframes.size());
    for (int i = 0; i < pointerframes.size() ; ++i) {
        temp[i] = cloud_to_cloud(pointerRef.markers(), pointerframes.at(i)) * pointerRef.tip();
        temp[i].noalias() = cloud_to_cloud(fixedRef.markers(),fixedframes.at(i)).inverse() * temp[i];
    }
    d.add_points(temp);
    return d;
}
