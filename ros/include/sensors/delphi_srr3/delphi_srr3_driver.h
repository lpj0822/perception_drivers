/*!
 *  \brief This file defines delphi srr3 ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLO_DELPHI_SRR3_DRIVER_H_
#define HOLO_DELPHI_SRR3_DRIVER_H_

#include <holo/sensors/delphi_srr3/delphi_srr3.h>
#include "holomatic/ObstacleDelphiSRR3.h"
#include "holomatic/ObstacleDelphiSRR3List.h"
#include <ros/ros.h>

namespace holomatic
{
namespace delphi_srr3
{

class DelphiSrr3Driver
{
public:
    DelphiSrr3Driver(ros::NodeHandle& nh);

private:
    bool GetParameters(ros::NodeHandle& nh);
    void DelphiSrr3RawObsHandler(ros::Publisher& publisher, const holo::sensors::delphi_srr3::DelphiSrr3ObjectList& input);

    std::shared_ptr<holo::sensors::delphi_srr3::DelphiSrr3> delphi_srr3_driver_;
    std::string yaml_file_, coord_;

    holomatic::ObstacleDelphiSRR3List delphi_srr3_obstacles_;
    ros::Publisher pub_delphi_srr3_[2];
};

}
}

#endif