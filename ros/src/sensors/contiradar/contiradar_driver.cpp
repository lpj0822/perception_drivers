/*!
 *  \brief conti 408 ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice.
 */

#include <sensors/contiradar/contiradar_driver.h>


namespace holomatic
{
namespace contiradar
{

ContiradarDriver::ContiradarDriver(ros::NodeHandle& nh)
{
    if(!GetParameters(nh))
    {
    	ROS_ERROR("Could not load parameters for contiradar driver");
    	return;
    }

    if (coord_ == "RADAR_FRONT_CENTER")
    {
        radar_index_ = holo::sensors::contiradar::Radar_location::FRONT;
        pub_conti_ = nh.advertise<holomatic::ObstacleConti408List>("/conti_obstacles/front_center", 1);
    }
    else if(coord_ == "RADAR_REAR_CENTER")
    {
        radar_index_ = holo::sensors::contiradar::Radar_location::REAR;
        pub_conti_ = nh.advertise<holomatic::ObstacleConti408List>("/conti_obstacles/rear_center", 1);
    }
    else
    {
        ROS_ERROR("For contiradar driver, could not find the corresponding coordinate");
        return;
    }

    contiradar_driver_ = std::make_shared<holo::sensors::contiradar::Contiradar>(yaml_file_);
    contiradar_driver_ -> InstallServiceRawObsCallback(std::bind(&ContiradarDriver::ContiradarRawObsHandler, this, std::placeholders::_1), radar_index_);
}

bool ContiradarDriver::GetParameters(ros::NodeHandle& nh)
{
	bool pass = true;
    pass &= nh.getParam("yaml_file", yaml_file_);
    pass &= nh.getParam("coord", coord_);

    return pass;
}

void ContiradarDriver::ConvertObstacle(const holo::sensors::contiradar::Ars048Obstacle& input,
        holomatic::ObstacleConti408& output)
{
    output.header.stamp.sec = input.timestamp.sec;
    output.header.stamp.nsec = input.timestamp.nsec;

    output.obj_id = input.obj_id;
    output.obj_dyn_prop = input.obj_dyn_prop;
    output.obj_dist_long = input.obj_dist_long;
    output.obj_dist_lat = input.obj_dist_lat;
    output.obj_vrel_long = input.obj_vrel_long;
    output.obj_vrel_lat = input.obj_vrel_lat;
    output.obj_arel_long = input.obj_arel_long;
    output.obj_arel_lat = input.obj_arel_lat;
    output.obj_dist_long_rms = input.obj_dist_long_rms;
    output.obj_dist_lat_rms = input.obj_dist_lat_rms;
    output.obj_vrel_long_rms = input.obj_vrel_long_rms;
    output.obj_vrel_lat_rms = input.obj_vrel_lat_rms;
    output.obj_arel_long_rms = input.obj_arel_long_rms;
    output.obj_arel_lat_rms = input.obj_arel_lat_rms;
    output.obj_rcs = input.obj_rcs;
    output.obj_meas_state = input.obj_meas_state;
    output.obj_prob_of_exist = input.obj_prob_of_exist;
    output.obj_class = input.obj_class;
    output.obj_orientation_angle = input.obj_orientation_angle;
    output.obj_orientation_rms = input.obj_orientation_rms;
    output.obj_length = input.obj_length;
    output.obj_width = input.obj_width;
}

void ContiradarDriver::ContiradarRawObsHandler(const holo::sensors::contiradar::Ars048ObstacleList& input)
{
    conti_obstacles_.obstacles.resize(input.obstacle_list.size());
    memset(&conti_obstacles_.obstacles[0], 0 , conti_obstacles_.obstacles.size());

    if (input.coord == holo::Coord::CONTI_FRONT_CENTER)
    {
        conti_obstacles_.header.frame_id = "CONTI_FRONT_CENTER";
    }
    else if (input.coord == holo::Coord::CONTI_REAR_CENTER)
    {
        conti_obstacles_.header.frame_id = "CONTI_REAR_CENTER";
    }
    else
    {
        conti_obstacles_.header.frame_id = "UNKNOWN";
    }
    conti_obstacles_.header.stamp.sec = input.timestamp.sec;
    conti_obstacles_.header.stamp.nsec = input.timestamp.nsec;

    for(int i = 0; i < input.obstacle_list.size(); i++)
    {
        ConvertObstacle(input.obstacle_list[i], conti_obstacles_.obstacles[i]);       
    }

    pub_conti_.publish(conti_obstacles_);
}   

}
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "contiradar_driver");

    ros::NodeHandle nh("~");

    holomatic::contiradar::ContiradarDriver node(nh);
    
    ros::spin();
    
    return 0;
}
