/*!
 *  \brief pandar ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice.
 */

#include <sensors/pandar/pandar_driver.h>

namespace holomatic
{
namespace pandar
{

PandarDriver::PandarDriver(ros::NodeHandle& nh)
{
    memset(&scan_, 0, sizeof(scan_));

    if(!GetParameters(nh))
    {
    	ROS_ERROR("Could not get parameters for pandar driver");
    	return;
    }
    driver_ = std::make_shared<holo::sensors::pandar::Pandar>(yaml_file_, data_port_, gps_port_, coord_);

    pub_pandar_scan_ = nh.advertise<holomatic::PandarScan>("/pandar/scan", 1);
    driver_ ->SetPandarScanCallback(std::bind(&PandarDriver::PandarScanHandler, this, std::placeholders::_1));
    driver_ ->Run();

}

bool PandarDriver::GetParameters(ros::NodeHandle& nh)
{
	bool pass = true;
    pass &= nh.getParam("yaml_file", yaml_file_);
    pass &= nh.getParam("coord", coord_);
    pass &= nh.getParam("data_port", data_port_);
    pass &= nh.getParam("gps_port", gps_port_);

    return pass;
}

void PandarDriver::ConvertBlock(holo::sensors::pandar::PandarBlock& input, holomatic::PandarBlock& output)
{
    output.rotation = input.rotation;
    memcpy(&output.data.at(0), &input.data[0], 192);
}


void PandarDriver::ConvertPacket(holo::sensors::pandar::PandarScan& input, holomatic::PandarScan& output)
{
    output.coord = input.coord;
    output.timestamp.sec = input.timestamp.sec;
    output.timestamp.nsec = input.timestamp.nsec;
    output.packets.resize(input.packets.size());

    for (size_t i = 0; i != input.packets.size(); i++)
    {
        output.packets.at(i).header = input.packets.at(i).header;
        output.packets.at(i).high_temperature = input.packets.at(i).high_temperature;
        output.packets.at(i).motor_speed = input.packets.at(i).motor_speed;
        output.packets.at(i).block_timestamp = input.packets.at(i).BlockTimeStamp();
        output.packets.at(i).return_mode = input.packets.at(i).return_mode;
        output.packets.at(i).factory_info = input.packets.at(i).factory_info;

        for (size_t j = 0; j != holo::sensors::pandar::PANDAR_BLOCKS_PER_PACKET; j++)
        {
            ConvertBlock(input.packets.at(i).blocks[j], output.packets.at(i).blocks.at(j));
        }
    }
}

void PandarDriver::PandarScanHandler(holo::sensors::pandar::PandarScan& input)
{
    ConvertPacket(input, scan_);
    pub_pandar_scan_.publish(scan_);
}

}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pandar_driver");

    ros::NodeHandle nh("~");

    holomatic::pandar::PandarDriver node(nh);
    
    ros::spin();
    
    return 0;
}
