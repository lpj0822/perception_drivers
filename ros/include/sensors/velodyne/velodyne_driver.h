/*!
 *  \brief This file defines velodyne ros driver
 *  \date Mar 20, 2018
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLOMATIC_VELODYNE_DRIVER_H_
#define HOLOMATIC_VELODYNE_DRIVER_H_

#include <holo/sensors/velodyne/velodyne.h>
#include "holomatic/VelodyneBlock.h"
#include "holomatic/VelodynePacket.h"
#include "holomatic/VelodyneScan.h"
#include "holomatic/VelodyneSync.h"
#include <ros/ros.h>

namespace holomatic
{
namespace velodyne
{

class VelodyneDriver
{

public:
	VelodyneDriver(ros::NodeHandle& nh);

private:
    bool GetParameters(ros::NodeHandle& nh);
    void ConvertBlock(holo::sensors::velodyne::VelodyneBlock& input, holomatic::VelodyneBlock& output);
    void Convert(holo::sensors::velodyne::VelodyneScan& input, holomatic::VelodyneScan& output);  
    void VelodyneScanHandler(holo::sensors::velodyne::VelodyneScan& input);

    std::shared_ptr<holo::sensors::velodyne::Velodyne> driver_;

    std::string yaml_file_, coord_;
    int data_port_, gps_port_;

    holomatic::VelodyneScan scan_;
    ros::Publisher pub_velodyne_scan_;
};

}
}

#endif