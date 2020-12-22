/*!
 *  \brief This file defines delphi esr ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLO_DELPHI_ESR_DRIVER_H_
#define HOLO_DELPHI_ESR_DRIVER_H_

#include <holo/sensors/delphi_esr/delphi_esr.h>
#include "holomatic/ObstacleDelphiESR.h"
#include "holomatic/ObstacleDelphiESRList.h"
#include <ros/ros.h>

namespace holomatic
{
namespace delphi_esr
{

class DelphiEsrDriver
{
public:
    DelphiEsrDriver(ros::NodeHandle& nh);

private:
    bool GetParameters(ros::NodeHandle& nh);
    void DelphiEsrRawObsHandler(const holo::sensors::delphi_esr::DelphiEsrObjectList& input);

    std::shared_ptr<holo::sensors::delphi_esr::DelphiEsr> delphi_esr_driver_;
    std::string yaml_file_, coord_;
    int radar_index_;
    std::string topic_name_;

    holomatic::ObstacleDelphiESRList delphi_esr_obstacles_;
    ros::Publisher pub_delphi_esr_;
};

}
}

#endif