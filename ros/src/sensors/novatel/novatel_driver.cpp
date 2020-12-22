/*!
 *  \brief novatel ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice.
 */

#include <sensors/novatel/novatel_driver.h>

namespace holomatic
{
namespace novatel
{

NovatelDriver::NovatelDriver(ros::NodeHandle& nh)
{
    if(!GetParameters(nh))
    {
    	ROS_ERROR("Could not load parameters for novate driver");
    }

    driver_ = std::make_shared<::holo::sensors::novatel::Novatel>(com_type_,ip_,port_,dev_, baud_rate_,cmd_);
    pub_inspvax_ = nh.advertise<holomatic::InsPositionVelocityAttitudeExtended>("/novatel/inspvax", 1);
    driver_ ->SetInsPvaXCallback(std::bind(
	            &NovatelDriver::InsPvaXHandler, this, std::placeholders::_1));
}

bool NovatelDriver::GetParameters(ros::NodeHandle& nh)
{
	bool pass = true;
    pass &= nh.getParam("dev", dev_);
    pass &= nh.getParam("baud_rate", baud_rate_);
    pass &= nh.getParam("coord", coord_);
    pass &= nh.getParam("com_type", com_type_);
    pass &= nh.getParam("ip", ip_);
    pass &= nh.getParam("port", port_);
    pass &= nh.getParam("command", cmd_);
    return pass;
}


void NovatelDriver::InsPvaXHandler(holo::sensors::novatel::InsPositionVelocityAttitudeExtended& input)
{
    holo::Timestamp ts = driver_-> ConvertGpsToTimestamp(input.header);
    inspvax_.header.stamp.sec = ts.sec;
    inspvax_.header.stamp.nsec = ts.nsec;
    inspvax_.header.frame_id = coord_;
    inspvax_.ins_status = input.ins_status;
    inspvax_.position_type = input.position_type;
    inspvax_.latitude = input.latitude;
    inspvax_.longitude = input.longitude;
    inspvax_.height = input.height;
    inspvax_.undulation = input.undulation;
    inspvax_.north_velocity = input.north_velocity;
    inspvax_.east_velocity = input.east_velocity;
    inspvax_.up_velocity = input.up_velocity;
    inspvax_.roll = input.roll;
    inspvax_.pitch = input.pitch;
    inspvax_.azimuth = input.azimuth;
    inspvax_.latitude_std = input.latitude_std;
    inspvax_.longitude_std = input.longitude_std;
    inspvax_.height_std = input.height_std;
    inspvax_.north_velocity_std = input.north_velocity_std;
    inspvax_.east_velocity_std = input.east_velocity_std;
    inspvax_.up_velocity_std = input.up_velocity_std;
    inspvax_.roll_std = input.roll_std;
    inspvax_.pitch_std = input.pitch_std;
    inspvax_.azimuth_std = input.azimuth_std;
    inspvax_.ext_solution_status = input.ext_solution_status;
    inspvax_.time_since_update = input.time_since_update;

    pub_inspvax_.publish(inspvax_);
}


}
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "novatel_driver");

    ros::NodeHandle nh("~");

    holomatic::novatel::NovatelDriver node(nh);
    
    ros::spin();
    
    return 0;
}
