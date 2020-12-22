/*!
* \brief This file defines velodyne lidar scan to pointcloud converter
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef _HOLO_SENSORS_VELODYNE_PCD_CONVERTER_H
#define _HOLO_SENSORS_VELODYNE_PCD_CONVERTER_H

#include <holo/yaml.h>
#include <holo/sensors/velodyne/velodyne_structure.h>

namespace holo
{
namespace sensors
{
namespace velodyne
{

class VelodynePcdConverter
{
public:
    // parameters for pcd conversion
    struct ConvertParameter
    {
         std::string  calibration_file;           // calibration file which is needed by vlp32 or hdl version
         bool grid;                               // if the pointcloud is needed to be grided
         float32_t min_dis, max_dis;              // the max and min distance of the pointcloud, if the point is not in this range, it would be filtered out
         float32_t start_azimuth, end_azimuth;    // the max and min angle of the pointcloud, if the point is not in this range, it would be filtered out (in vlp frame)
    };

    // the construction funtion
    // [input] yaml_file:     the yaml file loaded the convert parameters; the yaml file is in holo_sensors/parms/
    VelodynePcdConverter(const std::string& yaml_file);

    // get one frame of pointcloud from a velodyne scan
    // [input] scan:    the velodyne scan from velodyne driver
    // [output] pcd:    the point cloud got from scan
    size_t ParseScan(const VelodyneScan &scan, pcl::PointCloud<PointXYZIT> &pcd, double& stamp);

    // unpack the packet from vlp16 to get the point cloud from one packet
    // [input] packet:        one vlp16 packet from driver
    // [input] grid:          the parmeters in convert parameters, needed when set the dimension of pcd
    // [input] base_time:     the time hour of this scan, if the gps is used, the base_time is the gps hour or it would be 0
    // [input] offset:        because many packets would be unpacked in one pcd, this offset records the offset in pcd which is also the beginning of this unpack action
    // [output] pcd           the unpacked pcd
    size_t UnpackVLP16(
            const VelodynePacket& packet,
            const bool grid,
            const Timestamp &base_time,
            const size_t offset,
            pcl::PointCloud<PointXYZIT> &pcd) const;

private:


    // get the azimuth angle from the packet
    // [input] packet:       one vlp16 packet from driver
    // [output] azimuths:    the azimuth angle of packet, because for one packet has two 12 blocks and each block has two points which means there are two azimuth
    void GetPacketAzimuth(const VelodynePacket &packet, float32_t azimuths[][2]) const;
    
    // load the parameters from yaml file
    // [input] yaml_file: the yaml file loaded the convert parameters; the yaml file is in holo_sensors/parms/
    void LoadParameters(const std::string& yaml_file);
    
    // because for some reasons, there may be packets jumpping, rhis function is used to check the time sequence
    // [input] last_velodyne_timestamp:    the timestamp from last velodyne packet
    // [input] packets:                    one velodyne packet from driver
    // [input] base_hour:                  the base hour of this whole velodyne scan
    // [return]:                           return true if it is ok, return false if there is a huge jump
    bool CheckPacketFrame(const uint32_t last_velodyne_timestamp, const VelodynePacket &packet, Timestamp &base_hour) const;

    // convert parameters
    ConvertParameter convert_param_;
    // sin and cos values of vlp16, because there are 16 angles from it and they are constant
    float64_t sin_vertical_vlp16_[16];
    float64_t cos_vertical_vlp16_[16];
    // check if the pcd type has the field we need
    bool check_point_type_;
    bool has_intensity_, has_ring_, has_timestamp_, has_yaw_;
    pcl::PCLPointField x_field_, y_field_, z_field_, intensity_field_, ring_field_, timestamp_field_, yaw_field_;

};

}
}
}

#endif
