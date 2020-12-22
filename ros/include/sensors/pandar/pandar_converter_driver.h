/*!
 *  \brief This file defines pandar pcd converter ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLOMATIC_PANDAR_CONVERTER_DRIVER_H_
#define HOLOMATIC_PANDAR_CONVERTER_DRIVER_H_

#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <holo/sensors/pandar/pandar_pcd_converter.h>
#include <sensor_msgs/PointCloud2.h>
#include "holomatic/PandarBlock.h"
#include "holomatic/PandarPacket.h"
#include "holomatic/PandarScan.h"
#include "holomatic/PandarSync.h"
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace holomatic
{
namespace pandar
{

class PandarConverterDriver
{

public:
    PandarConverterDriver(ros::NodeHandle& nh);

private:
    bool GetParameters(ros::NodeHandle& nh);
    void ConvertPandarBlock(const holomatic::PandarBlock& input,
             holo::sensors::pandar::PandarBlock& output);
    void ConvertPandarScan(const holomatic::PandarScan& input,
             holo::sensors::pandar::PandarScan& output);
    void CallbackPandarScan(const holomatic::PandarScan& input);

    ros::Subscriber sub_pandar_scan_;
    ros::Publisher pub_pcd_;

    std::shared_ptr<holo::sensors::pandar::PandarPcdConverter> pandar_pcd_converter_;

    std::string yaml_file_, sub_topic_name_, pub_topic_name_;
};

}
}

#endif
