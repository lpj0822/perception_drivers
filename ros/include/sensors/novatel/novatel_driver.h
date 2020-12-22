/*!
 *  \brief This file defines novatel ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLOMATIC_NOVATEL_DRIVER_H_
#define HOLOMATIC_NOVATEL_DRIVER_H_

#include <holo/sensors/novatel/novatel.h>
#include "holomatic/InsPositionVelocityAttitudeExtended.h"
#include <ros/ros.h>

namespace holomatic
{
namespace novatel
{

class NovatelDriver
{
public:

    NovatelDriver(ros::NodeHandle& nh);

private:
    bool GetParameters(ros::NodeHandle& nh);

    void InsPvaXHandler(holo::sensors::novatel::InsPositionVelocityAttitudeExtended& input);

    std::shared_ptr<holo::sensors::novatel::Novatel> driver_;

    std::string dev_, coord_,ip_,com_type_,cmd_;
    int baud_rate_,port_;

    holomatic::InsPositionVelocityAttitudeExtended inspvax_;
    ros::Publisher pub_inspvax_;
};

}
}

#endif
