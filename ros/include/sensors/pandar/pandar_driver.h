/*!
 *  \brief This file defines pandar ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLOMATIC_PANDAR_DRIVER_H_
#define HOLOMATIC_PANDAR_DRIVER_H_

#include <holo/sensors/pandar/pandar.h>
#include "holomatic/PandarBlock.h"
#include "holomatic/PandarPacket.h"
#include "holomatic/PandarScan.h"
#include "holomatic/PandarSync.h"
#include <ros/ros.h>

namespace holomatic
{
namespace pandar
{

class PandarDriver
{

public:
	PandarDriver(ros::NodeHandle& nh);

private:
    bool GetParameters(ros::NodeHandle& nh);
    void ConvertBlock(holo::sensors::pandar::PandarBlock& input, holomatic::PandarBlock& output);
    void ConvertPacket(holo::sensors::pandar::PandarScan& input, holomatic::PandarScan& output);
    void PandarScanHandler(holo::sensors::pandar::PandarScan& input);

    std::shared_ptr<holo::sensors::pandar::Pandar> driver_;

    std::string yaml_file_, coord_;
    int data_port_, gps_port_;

    holomatic::PandarScan scan_;
    ros::Publisher pub_pandar_scan_;
};

}
}

#endif