/*!
 *  \brief velodyne pcd converter ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice.
 */

#include <sensors/velodyne/velodyne_converter_driver.h>
#include <glog/logging.h>

namespace holomatic
{
namespace velodyne
{

VelodyneConverterDriver::VelodyneConverterDriver(ros::NodeHandle& nh)
{
    if(!GetParameters(nh))
    {
        std::cout << "Could not load parameters";
        return;
    }

    pub_pcd_ = nh.advertise<sensor_msgs::PointCloud2>(pub_topic_name_, 1);

    velodyne_pcd_converter_ = std::make_shared<holo::sensors::velodyne::VelodynePcdConverter>(yaml_file_);

    sub_velodyne_scan_ = nh.subscribe(sub_topic_name_, 1, &VelodyneConverterDriver::CallbackVelodyneScan, this);
}

bool VelodyneConverterDriver::GetParameters(ros::NodeHandle& nh)
{
    bool pass = true;
    pass &= nh.getParam("yaml_file", yaml_file_);
    pass &= nh.getParam("sub_topic_name", sub_topic_name_);
    pass &= nh.getParam("pub_topic_name", pub_topic_name_);

    return pass;
}

void VelodyneConverterDriver::ConvertVelodyneBlock(const holomatic::VelodyneBlock& input,
             holo::sensors::velodyne::VelodyneBlock& output)
{
    output.header = input.header;
    output.rotation = input.rotation;
    memcpy(&output.data[0], &input.data.at(0), 96);
}

void VelodyneConverterDriver::ConvertVelodyneScan(const holomatic::VelodyneScan& input,
             holo::sensors::velodyne::VelodyneScan& output)
{
    output.coord_id = input.coord;
    output.timestamp.sec = input.timestamp.sec;
    output.timestamp.nsec = input.timestamp.nsec;
    output.packets.resize(input.packets.size());
    for (size_t i = 0; i != input.packets.size(); i++)
    {
        output.packets.at(i).velodyne_timestamp = input.packets.at(i).velodyne_timestamp;
        output.packets.at(i).reserved = input.packets.at(i).reserved;
        for (size_t j = 0; j != holo::sensors::velodyne::VELODYNE_BLOCKS_PER_PACKET; j++)
        {
            ConvertVelodyneBlock(input.packets.at(i).blocks.at(j),
                    output.packets.at(i).blocks[j]);
        }
    }
}

void VelodyneConverterDriver::CallbackVelodyneScan(const holomatic::VelodyneScan& input)
{
    holo::sensors::velodyne::VelodyneScan scan;
    ConvertVelodyneScan(input, scan);
    pcl::PointCloud<PointXYZIT> pcd;
    holo::float64_t stamp;
    velodyne_pcd_converter_ -> ParseScan(scan, pcd, stamp);
    sensor_msgs::PointCloud2 output;
    pcl_conversions::toPCL(ros::Time(stamp), pcd.header.stamp);
    pcl::toROSMsg(pcd, output);
    pub_pcd_.publish(output);
}

}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_converter_driver");

    ros::NodeHandle nh("~");

    holomatic::velodyne::VelodyneConverterDriver node(nh);
    
    ros::spin();
    
    return 0;
}
