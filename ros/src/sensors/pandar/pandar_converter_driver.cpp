/*!
 *  \brief pandar pcd converter ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice.
 */

#include <sensors/pandar/pandar_converter_driver.h>
#include <glog/logging.h>
#include <iostream>

namespace holomatic
{
namespace pandar
{

PandarConverterDriver::PandarConverterDriver(ros::NodeHandle& nh)
{
    if(!GetParameters(nh))
    {
        std::cout << "Could not load parameters";
        return;
    }

    pub_pcd_ = nh.advertise<sensor_msgs::PointCloud2>(pub_topic_name_, 1);

    pandar_pcd_converter_ = std::make_shared<holo::sensors::pandar::PandarPcdConverter>(yaml_file_);

    sub_pandar_scan_ = nh.subscribe(sub_topic_name_, 1, &PandarConverterDriver::CallbackPandarScan, this);
}

bool PandarConverterDriver::GetParameters(ros::NodeHandle& nh)
{
    bool pass = true;
    pass &= nh.getParam("yaml_file", yaml_file_);
    pass &= nh.getParam("sub_topic_name", sub_topic_name_);
    pass &= nh.getParam("pub_topic_name", pub_topic_name_);

    return pass;
}

void PandarConverterDriver::ConvertPandarBlock(const holomatic::PandarBlock& input,
             holo::sensors::pandar::PandarBlock& output)
{
    output.rotation = input.rotation;
    memcpy(&output.data[0], &input.data.at(0), 192);
}

void PandarConverterDriver::ConvertPandarScan(const holomatic::PandarScan& input,
             holo::sensors::pandar::PandarScan& output)
{
    output.coord = input.coord;
    output.timestamp.sec = input.timestamp.sec;
    output.timestamp.nsec = input.timestamp.nsec;
    output.packets.resize(input.packets.size());

    for (size_t i = 0; i != input.packets.size(); i++)
    {
        output.packets.at(i).block_timestamp[3] =
            (input.packets.at(i).block_timestamp >> 24) & 0xFF;
        output.packets.at(i).block_timestamp[2] =
            (input.packets.at(i).block_timestamp >> 16) & 0xFF;
        output.packets.at(i).block_timestamp[1] =
            (input.packets.at(i).block_timestamp >> 8) & 0xFF;
        output.packets.at(i).block_timestamp[0] =
            (input.packets.at(i).block_timestamp) & 0xFF;
        output.packets.at(i).high_temperature =
            input.packets.at(i).high_temperature;
        output.packets.at(i).motor_speed = input.packets.at(i).motor_speed;
        output.packets.at(i).return_mode = input.packets.at(i).return_mode;
        output.packets.at(i).factory_info = input.packets.at(i).factory_info;
        output.packets.at(i).header = input.packets.at(i).header;
        for (size_t j = 0; j != 6; j++)
        {
            ConvertPandarBlock(input.packets.at(i).blocks.at(j),
                    output.packets.at(i).blocks[j]);
        }
    }
}

void PandarConverterDriver::CallbackPandarScan(const holomatic::PandarScan& input)
{
    holo::sensors::pandar::PandarScan scan;
    ConvertPandarScan(input, scan);
    pcl::PointCloud<PointXYZIT> pcd;
    holo::float64_t stamp;
    pandar_pcd_converter_ -> ParseScan(scan, pcd, stamp);
    sensor_msgs::PointCloud2 output;
    pcl_conversions::toPCL(ros::Time(stamp), pcd.header.stamp);
    std::cout << "stamp:"<< std::setprecision(32) << pcd.header.stamp / 1000000.0 << std::endl;
    pcl::toROSMsg(pcd, output);
    pub_pcd_.publish(output);
}

}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pandar_converter_driver");

    ros::NodeHandle nh("~");

    holomatic::pandar::PandarConverterDriver node(nh);
    
    ros::spin();
    
    return 0;
}
