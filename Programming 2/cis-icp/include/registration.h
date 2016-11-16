/**
 * @author Doran Walsten, Ravi Gaddipati
 * @date Nov 15, 2016
 *
 * @brief
 * Registration between fixed and mobile bodies.
 *
 * @file
 */

#ifndef CIS_ICP_REGISTRATION_H
#define CIS_ICP_REGISTRATION_H

#include "files.h"
#include "pointcloud.h"

namespace cis {

    /**
     * This function is used to convert transform the tip of the pointer from the pointer frame to the fixed body frame
     * This is done by using the RigidBody data as the reference frame within our point cloud transform method to determine
     * the registration for each frame. Then, this is applied to the tip vector.
     * @param pointerRef
     * @param fixedRef
     * @param pointerframes
     * @param fixedframes
     * @return PointCloud
     */
    cis::PointCloud pointer_to_fixed(const cis::RigidBody &pointerRef,
                                     const cis::RigidBody &fixedRef,
                                     const std::vector<cis::PointCloud> &pointerframes,
                                     const std::vector<cis::PointCloud> &fixedframes);
}
#endif //CIS_ICP_REGISTRATION_H
