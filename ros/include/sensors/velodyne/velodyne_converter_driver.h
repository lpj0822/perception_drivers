/*!
 *  \brief This file defines velodyne pcd converter ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLOMATIC_VELODYNE_CONVERTER_DRIVER_H_
#define HOLOMATIC_VELODYNE_CONVERTER_DRIVER_H_

#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <holo/sensors/velodyne/velodyne_pcd_converter.h>
#include <sensor_msgs/PointCloud2.h>
#include "holomatic/VelodyneBlock.h"
#include "holomatic/VelodynePacket.h"
#include "holomatic/VelodyneScan.h"
#include "holomatic/VelodyneSync.h"
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace holomatic
{
namespace velodyne
{

class VelodyneConverterDriver
{

public:
    VelodyneConverterDriver(ros::NodeHandle& nh);

private:
    bool GetParameters(ros::NodeHandle& nh);
    void ConvertVelodyneBlock(const holomatic::VelodyneBlock& input,
             holo::sensors::velodyne::VelodyneBlock& output);
    void ConvertVelodyneScan(const holomatic::VelodyneScan& input,
             holo::sensors::velodyne::VelodyneScan& output);   
    void CallbackVelodyneScan(const holomatic::VelodyneScan& input);

    ros::Subscriber sub_velodyne_scan_;
    ros::Publisher pub_pcd_;

    std::shared_ptr<holo::sensors::velodyne::VelodynePcdConverter> velodyne_pcd_converter_;

    std::string yaml_file_, sub_topic_name_, pub_topic_name_;
};

}
}

#endif
