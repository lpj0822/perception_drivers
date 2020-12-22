/*!
 *  \brief delphi srr3 ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice.
 */

#include <sensors/delphi_srr3/delphi_srr3_driver.h>

namespace holomatic
{
namespace delphi_srr3
{

DelphiSrr3Driver::DelphiSrr3Driver(ros::NodeHandle& nh)
{
    if(!GetParameters(nh))
    {
    	ROS_ERROR("Could not load parameters for delphi srr3 driver");
    	return;
    }

    delphi_srr3_driver_ = std::make_shared<holo::sensors::delphi_srr3::DelphiSrr3>(yaml_file_);

    if (coord_ == "RADAR_FRONT_CORNER")
    {
        pub_delphi_srr3_[0] = nh.advertise<holomatic::ObstacleDelphiSRR3List>("/delphi_srr3_obstacles/front_left", 1);
        pub_delphi_srr3_[1] = nh.advertise<holomatic::ObstacleDelphiSRR3List>("/delphi_srr3_obstacles/front_right", 1);
    }
    else if (coord_ == "RADAR_REAR_CORNER")
    {
        pub_delphi_srr3_[0] = nh.advertise<holomatic::ObstacleDelphiSRR3List>("/delphi_srr3_obstacles/rear_left", 1);
        pub_delphi_srr3_[1] = nh.advertise<holomatic::ObstacleDelphiSRR3List>("/delphi_srr3_obstacles/rear_right", 1);
    }
    else
    {
        ROS_ERROR("For delphi srr3 driver, could not find the corresponding coordinate");
        return;
    }   

    delphi_srr3_driver_ -> InstallServiceRawObsCallback(std::bind(&DelphiSrr3Driver::DelphiSrr3RawObsHandler, this, 
            std::ref(pub_delphi_srr3_[0]), std::placeholders::_1), 0);
    delphi_srr3_driver_ -> InstallServiceRawObsCallback(std::bind(&DelphiSrr3Driver::DelphiSrr3RawObsHandler, this, 
            std::ref(pub_delphi_srr3_[1]), std::placeholders::_1), 1);     
}

bool DelphiSrr3Driver::GetParameters(ros::NodeHandle& nh)
{
	bool pass = true;
    pass &= nh.getParam("yaml_file", yaml_file_);
    pass &= nh.getParam("coord", coord_);

    return pass;
}

void DelphiSrr3Driver::DelphiSrr3RawObsHandler(ros::Publisher& publisher,
        const holo::sensors::delphi_srr3::DelphiSrr3ObjectList& input)
{
    delphi_srr3_obstacles_.obstacles.clear();
    delphi_srr3_obstacles_.obstacles.resize(15);
    memset(&delphi_srr3_obstacles_.obstacles[0], 0 , 15);

    delphi_srr3_obstacles_.header.stamp.sec = input.timestamp.sec;
    delphi_srr3_obstacles_.header.stamp.nsec = input.timestamp.nsec;
    if(input.coord_id == holo::Coord::DELPHI_SRR3_FRONT_LEFT)
    {
        delphi_srr3_obstacles_.header.frame_id = "DELPHI_SRR3_FRONT_LEFT";
    }
    else if(input.coord_id == holo::Coord::DELPHI_SRR3_FRONT_RIGHT)
    {
        delphi_srr3_obstacles_.header.frame_id = "DELPHI_SRR3_FRONT_RIGHT";
    }
    else if(input.coord_id == holo::Coord::DELPHI_SRR3_REAR_LEFT)
    {
        delphi_srr3_obstacles_.header.frame_id = "DELPHI_SRR3_REAR_LEFT";
    }
    else if(input.coord_id == holo::Coord::DELPHI_SRR3_REAR_RIGHT)
    {
        delphi_srr3_obstacles_.header.frame_id = "DELPHI_SRR3_REAR_RIGHT";
    }
    else
    {
        ROS_ERROR("could not find the corresponding coordinate");
    }

    for (uint32_t i = 0; i < 15; i++)
    {
        delphi_srr3_obstacles_.obstacles[i].header.stamp.sec = input.objects[i].timestamp.sec;
        delphi_srr3_obstacles_.obstacles[i].header.stamp.nsec = input.objects[i].timestamp.nsec;
        if(input.coord_id == holo::Coord::DELPHI_SRR3_FRONT_LEFT)
        {
            delphi_srr3_obstacles_.obstacles[i].header.frame_id = "DELPHI_SRR3_FRONT_LEFT";
        }
        else if(input.coord_id == holo::Coord::DELPHI_SRR3_FRONT_RIGHT)
        {
            delphi_srr3_obstacles_.obstacles[i].header.frame_id = "DELPHI_SRR3_FRONT_RIGHT";
        }
        else if(input.coord_id == holo::Coord::DELPHI_SRR3_REAR_LEFT)
        {
            delphi_srr3_obstacles_.obstacles[i].header.frame_id = "DELPHI_SRR3_REAR_LEFT";
        }
        else if(input.coord_id == holo::Coord::DELPHI_SRR3_REAR_RIGHT)
        {
            delphi_srr3_obstacles_.obstacles[i].header.frame_id = "DELPHI_SRR3_REAR_RIGHT";
        }
        delphi_srr3_obstacles_.obstacles[i].obj_id = input.objects[i].obj_id;
        delphi_srr3_obstacles_.obstacles[i].object_motion_pattern = input.objects[i].object_motion_pattern;
        delphi_srr3_obstacles_.obstacles[i].object_type = input.objects[i].object_type;
        delphi_srr3_obstacles_.obstacles[i].alive_time = input.objects[i].alive_time;
        delphi_srr3_obstacles_.obstacles[i].confidence = input.objects[i].confidence;
        delphi_srr3_obstacles_.obstacles[i].longitudinal_position = input.objects[i].longitudinal_position;
        delphi_srr3_obstacles_.obstacles[i].lateral_position = input.objects[i].lateral_position;
        delphi_srr3_obstacles_.obstacles[i].longitudinal_velocity = input.objects[i].longitudinal_velocity;
        delphi_srr3_obstacles_.obstacles[i].lateral_velocity = input.objects[i].lateral_velocity;
        delphi_srr3_obstacles_.obstacles[i].longitudinal_acceleration = input.objects[i].longitudinal_acceleration;
        delphi_srr3_obstacles_.obstacles[i].lateral_acceleration = input.objects[i].lateral_acceleration;
        delphi_srr3_obstacles_.obstacles[i].width = input.objects[i].width;
        delphi_srr3_obstacles_.obstacles[i].heading = input.objects[i].heading;
        delphi_srr3_obstacles_.obstacles[i].is_coasted = input.objects[i].is_coasted;
    }

    publisher.publish(delphi_srr3_obstacles_);
}

}
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "delphi_srr3_driver");

    ros::NodeHandle nh("~");

    holomatic::delphi_srr3::DelphiSrr3Driver node(nh);
    
    ros::spin();
    
    return 0;
}
