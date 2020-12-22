/*!
 *  \brief This file defines point cloud class
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLO_COMMON_POINT_CLOUD_H_
#define HOLO_COMMON_POINT_CLOUD_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

struct PointXYZIT 
{
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    uint16_t ring;
    float yaw;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIT,           
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (double, timestamp, timestamp)
                                   (uint16_t, ring, ring)
                                   (float, yaw, yaw)
)

#endif
