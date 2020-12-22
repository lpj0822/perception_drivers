/*!
 *  \brief velodyne ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice.
 */

#include <sensors/velodyne/velodyne_driver.h>

namespace holomatic
{
namespace velodyne
{

VelodyneDriver::VelodyneDriver(ros::NodeHandle& nh)
{
    memset(&scan_, 0, sizeof(scan_));

    if(!GetParameters(nh))
    {
    	ROS_ERROR("Could not get parameters for vlp16 driver");
    	return;
    }
    driver_ = std::make_shared<holo::sensors::velodyne::Velodyne>(yaml_file_, data_port_, gps_port_, coord_);

    std::string topic_name;

    if (coord_ == "VLP_FRONT_CENTER")
    {
        topic_name = "/velodyne/scan/center";
    }
    else if (coord_ == "VLP_FRONT_LEFT")
    {
        topic_name = "/velodyne/scan/left";
    }
    else if (coord_ == "VLP_FRONT_RIGHT")
    {
        topic_name = "/velodyne/scan/right";
    }
    else
    {
        /* TODO add other coord */
    }

    pub_velodyne_scan_ = nh.advertise<holomatic::VelodyneScan>(topic_name, 1);
    driver_ ->SetVelodyneScanCallback(std::bind(&VelodyneDriver::VelodyneScanHandler, this, std::placeholders::_1));
    driver_ ->Run();

}

bool VelodyneDriver::GetParameters(ros::NodeHandle& nh)
{
	bool pass = true;
    pass &= nh.getParam("yaml_file", yaml_file_);
    pass &= nh.getParam("coord", coord_);
    pass &= nh.getParam("data_port", data_port_);
    pass &= nh.getParam("gps_port", gps_port_);

    return pass;
}

void VelodyneDriver::ConvertBlock(holo::sensors::velodyne::VelodyneBlock& input, holomatic::VelodyneBlock& output)
{
    output.header = input.header;
    output.rotation = input.rotation;
    memcpy(&output.data.at(0), &input.data[0], 96);
}


void VelodyneDriver::Convert(holo::sensors::velodyne::VelodyneScan& input, holomatic::VelodyneScan& output)
{
    output.coord = input.coord_id;
    output.timestamp.sec = input.timestamp.sec;
    output.timestamp.nsec = input.timestamp.nsec;
    output.packets.resize(input.packets.size());

    for (size_t i = 0; i != input.packets.size(); i++)
    {
        output.packets.at(i).velodyne_timestamp = input.packets.at(i).velodyne_timestamp;
        output.packets.at(i).reserved = input.packets.at(i).reserved;

        for (size_t j = 0; j != holo::sensors::velodyne::VELODYNE_BLOCKS_PER_PACKET; j++)
        {
            ConvertBlock(input.packets.at(i).blocks[j], output.packets.at(i).blocks.at(j));
        }
    }
}

void VelodyneDriver::VelodyneScanHandler(holo::sensors::velodyne::VelodyneScan& input)
{
    Convert(input, scan_);
    pub_velodyne_scan_.publish(scan_);
}

}
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "velodyne_driver");

    ros::NodeHandle nh("~");

    holomatic::velodyne::VelodyneDriver node(nh);
    
    ros::spin();
    
    return 0;
}
