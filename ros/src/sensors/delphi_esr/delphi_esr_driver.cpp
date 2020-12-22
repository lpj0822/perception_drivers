/*!
 *  \brief delphi esr ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice.
 */

#include <sensors/delphi_esr/delphi_esr_driver.h>

namespace holomatic
{
namespace delphi_esr
{

DelphiEsrDriver::DelphiEsrDriver(ros::NodeHandle& nh)
{
    if(!GetParameters(nh))
    {
    	ROS_ERROR("Could not load parameters for delphi esr driver");
    	return;
    }

    if (coord_ == "RADAR_FRONT_CENTER")
    {
        radar_index_ = holo::sensors::delphi_esr::DelphiEsrLocation::FRONT_CENTER;
        topic_name_ = "/delphi_esr_obstacles/front_center";
    }
    else if (coord_ == "RADAR_REAR_CENTER")
    {
        radar_index_ = holo::sensors::delphi_esr::DelphiEsrLocation::REAR_CENTER;
        topic_name_ = "/delphi_esr_obstacles/rear_center";
    }
    else
    {
        ROS_ERROR("For delphi esr driver, could not find the corresponding coordinate");
        return;
    }

    pub_delphi_esr_ = nh.advertise<holomatic::ObstacleDelphiESRList>(topic_name_, 1);

    delphi_esr_driver_ = std::make_shared<holo::sensors::delphi_esr::DelphiEsr>(yaml_file_);
    delphi_esr_driver_ -> InstallServiceRawObsCallback(std::bind(&DelphiEsrDriver::DelphiEsrRawObsHandler, this, std::placeholders::_1), radar_index_);
}

bool DelphiEsrDriver::GetParameters(ros::NodeHandle& nh)
{
	bool pass = true;
    pass &= nh.getParam("yaml_file", yaml_file_);
    pass &= nh.getParam("coord", coord_);

    return pass;
}

void DelphiEsrDriver::DelphiEsrRawObsHandler(const holo::sensors::delphi_esr::DelphiEsrObjectList& input)
{
    delphi_esr_obstacles_.obstacles.clear();
    delphi_esr_obstacles_.obstacles.resize(64);
    memset(&delphi_esr_obstacles_.obstacles[0], 0 , 64);

    delphi_esr_obstacles_.header.stamp.sec = input.timestamp.sec;
    delphi_esr_obstacles_.header.stamp.nsec = input.timestamp.nsec;
    if(input.coord_id == holo::Coord::DELPHI_ESR_FRONT_CENTER)
    {
        delphi_esr_obstacles_.header.frame_id = "DELPHI_ESR_FRONT_CENTER";
    }
    else if(input.coord_id == holo::Coord::DELPHI_ESR_REAR_CENTER)
    {
        delphi_esr_obstacles_.header.frame_id = "DELPHI_ESR_REAR_CENTER";
    }

    for (uint32_t i = 0; i < 64; i++)
    {
        delphi_esr_obstacles_.obstacles[i].header.stamp.sec = input.objects[i].timestamp.sec;
        delphi_esr_obstacles_.obstacles[i].header.stamp.nsec = input.objects[i].timestamp.nsec;
        if(input.coord_id == holo::Coord::DELPHI_ESR_FRONT_CENTER)
        {
            delphi_esr_obstacles_.obstacles[i].header.frame_id = "DELPHI_ESR_FRONT_CENTER";
        }
        else if(input.coord_id == holo::Coord::DELPHI_ESR_REAR_CENTER)
        {
            delphi_esr_obstacles_.obstacles[i].header.frame_id = "DELPHI_ESR_REAR_CENTER";
        }
        delphi_esr_obstacles_.obstacles[i].obj_id = input.objects[i].obj_id;
        delphi_esr_obstacles_.obstacles[i].med_range_mode = input.objects[i].med_range_mode;
        delphi_esr_obstacles_.obstacles[i].status = input.objects[i].status;
        delphi_esr_obstacles_.obstacles[i].angle = input.objects[i].angle;
        delphi_esr_obstacles_.obstacles[i].range = input.objects[i].range;
        delphi_esr_obstacles_.obstacles[i].range_rate = input.objects[i].range_rate;
        delphi_esr_obstacles_.obstacles[i].range_accel = input.objects[i].range_accel;
        delphi_esr_obstacles_.obstacles[i].lat_rate = input.objects[i].lat_rate;
        delphi_esr_obstacles_.obstacles[i].width = input.objects[i].width;
        delphi_esr_obstacles_.obstacles[i].is_bridge = input.objects[i].is_bridge;
        delphi_esr_obstacles_.obstacles[i].is_oncoming = input.objects[i].is_oncoming;
        delphi_esr_obstacles_.obstacles[i].is_grouping_changed = input.objects[i].is_grouping_changed;
    }

    pub_delphi_esr_.publish(delphi_esr_obstacles_);
}

}
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "delphi_esr_driver");

    ros::NodeHandle nh("~");

    holomatic::delphi_esr::DelphiEsrDriver node(nh);
    
    ros::spin();
    
    return 0;
}
