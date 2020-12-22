/*!
 *  \brief This file defines conti 408 ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLOMATIC_CONTIRADAR_DRIVER_H_
#define HOLOMATIC_CONTIRADAR_DRIVER_H_

#include <holo/sensors/contiradar/contiradar.h>
#include "holomatic/ObstacleConti408.h"
#include "holomatic/ObstacleConti408List.h"
#include <ros/ros.h>

namespace holomatic
{
namespace contiradar
{

class ContiradarDriver
{
public:
    ContiradarDriver(ros::NodeHandle& nh);

private:
    bool GetParameters(ros::NodeHandle& nh);
    void ContiradarRawObsHandler(const holo::sensors::contiradar::Ars048ObstacleList& input);
	void ConvertObstacle(const holo::sensors::contiradar::Ars048Obstacle& input, holomatic::ObstacleConti408& output);

    std::shared_ptr<holo::sensors::contiradar::Contiradar> contiradar_driver_;
    std::string yaml_file_, coord_;
    int radar_index_;

    holomatic::ObstacleConti408List conti_obstacles_;
    ros::Publisher pub_conti_;
};

}
}

#endif
