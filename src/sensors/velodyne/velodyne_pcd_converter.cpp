/*!
* \brief This file defines velodyne lidar scan to pointcloud converter
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#include <holo/sensors/velodyne/velodyne_pcd_converter.h>
#include <glog/logging.h>

namespace holo
{
namespace sensors
{
namespace velodyne
{

VelodynePcdConverter::VelodynePcdConverter(const std::string& yaml_file)
{
    // the angles of 16 beams from vlp16 read from manual
    const int vertical_angle_vlp16_[16] = { -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};
    //generate the angle array, will be used in the conversion
    for (int i = 0; i < 16; i++)
    {
        sin_vertical_vlp16_[i] = std::sin(vertical_angle_vlp16_[i] * M_PI / 180);
        cos_vertical_vlp16_[i] = std::cos(vertical_angle_vlp16_[i] * M_PI / 180);
    }
    has_intensity_ = false;
    has_ring_ = false;
    has_timestamp_ = false;
    has_yaw_ = false;
    check_point_type_ = false;
    LoadParameters(yaml_file);
}

void VelodynePcdConverter::LoadParameters(const std::string& yaml_file)
{
    yaml::Node root_node;
    root_node = yaml::LoadFile(yaml_file);
    convert_param_.calibration_file = root_node["calibration_file"].as<std::string>();
    convert_param_.grid = root_node["grid"].as<bool>();
    convert_param_.min_dis = root_node["min_dis"].as<float32_t>();
    convert_param_.max_dis = root_node["max_dis"].as<float32_t>();
    convert_param_.start_azimuth = root_node["start_azimuth"].as<float32_t>();
    convert_param_.end_azimuth = root_node["end_azimuth"].as<float32_t>();
    convert_param_.start_azimuth = convert_param_.start_azimuth > 360 ? 360 : convert_param_.start_azimuth;
    convert_param_.start_azimuth = convert_param_.start_azimuth < 0 ? 0 : convert_param_.start_azimuth;
    convert_param_.end_azimuth = convert_param_.end_azimuth > 360 ? 360 : convert_param_.end_azimuth;
    convert_param_.end_azimuth = convert_param_.end_azimuth < 0 ? 0 : convert_param_.end_azimuth;
}

void VelodynePcdConverter::GetPacketAzimuth(const VelodynePacket &packet, float32_t azimuths[][2]) const
{
    float32_t azimuth_diff = 0.0;
    for (int block = 0; block < VELODYNE_BLOCKS_PER_PACKET; block++)
    {
        //the rotation for the first sector is able to be obtained, but for the second sector, the rotation has to be caculated
        azimuths[block][0] = packet.Block(block).Azimuth();

        if (block < (VELODYNE_BLOCKS_PER_PACKET - 1))
        {
            float32_t next_azimuth = packet.blocks[block + 1].Azimuth();

            if (azimuths[block][0] > next_azimuth)
            {
                next_azimuth = next_azimuth + 360.0;
            }

            //the rotation of the second sector is obtained here
            azimuths[block][1] = azimuths[block][0] + (next_azimuth - azimuths[block][0]) / 2;

            //get azimuth_diff for the last block
            azimuth_diff = azimuths[block][1] - azimuths[block][0];
        }
        else
        {
            azimuths[block][1] = azimuths[block][0] + azimuth_diff;
        }
    }
}

bool VelodynePcdConverter::CheckPacketFrame(const uint32_t last_velodyne_timestamp,
        const VelodynePacket &packet,
        Timestamp &base_hour) const
{
    bool proceed = true;
    // check if there is frame skipping when the packets are transmitting
    if ( packet.velodyne_timestamp >= last_velodyne_timestamp )
    {
        // if the forward jump is bigger than 0.1 second, we raise a warning and ignore it
        const float64_t d = (float64_t)packet.velodyne_timestamp - (float64_t)last_velodyne_timestamp;
        if ( d <= 0.1 * 1000000) // the normal situation the gap between two packets are smaller than 0.1 sec
        {
            proceed = true;
        }
        else if ( (d > 0.1 * 1000000) && (d < 3597.0*1000000) )
        {
            LOG(INFO) << "velodyne converter detects major forward timestamp jitter! --> "
                         << last_velodyne_timestamp << ", "
                         << packet.velodyne_timestamp << ", "
                         << d
                         << std::endl;
            proceed = false;
        }
        else
        {
            // maybe there is a packet from last hour
            base_hour = Add(base_hour, NormalizeDuration(-3600,0));
            proceed = true;
        }
    }
    else
    {
        // if the backward jump is less than 0.1 seconds, we proceed as normal
        const double d = (double)last_velodyne_timestamp - (double)packet.velodyne_timestamp;
        if ( d <= (0.1*1000000))
        {
            LOG(INFO) << "velodyne converter detects minor backward timestamp jitter! --> "
                         << last_velodyne_timestamp << ", " << packet.velodyne_timestamp << ", " << d
                         << std::endl;
            proceed = true;
        }
        // if the backward jump is between 0.1-3597 seconds, this is a serious problem
        else if ( (d > (0.1*1000000)) && (d <= (3597.0*1000000)) )
        {
            LOG(WARNING) << "velodyne converter detects major backward timestamp jitter! --> "
                       << last_velodyne_timestamp << ", " << packet.velodyne_timestamp << ", " << d
                       << std::endl;
            proceed = false;
        }
        // if the backward jump is between 3595-3600 seconds, we say this is a reasonable backward jump
        else
        {
            // if a new hour is started, another 3600 seconds would add to the time out of hour
            base_hour = Add(base_hour, NormalizeDuration(3600,0));
            proceed = true;
        }
    }
    return proceed;
}

size_t VelodynePcdConverter::ParseScan(const VelodyneScan &scan, pcl::PointCloud<PointXYZIT> &pcd, double& stamp)
{
    if(!check_point_type_)
    {
       std::vector<pcl::PCLPointField> fields;
       pcl::getFields<PointXYZIT>(fields);
       bool has_x = false;
       bool has_y = false;
       bool has_z = false;
       for(size_t i=0; i != fields.size(); i++)
       {
           if(fields[i].name == "x" && fields[i].datatype == pcl::PCLPointField::FLOAT32)
           {
              has_x = true;
              x_field_ = fields[i];
           }
           else if(fields[i].name == "y" && fields[i].datatype == pcl::PCLPointField::FLOAT32)
           {
              has_y = true;
              y_field_ = fields[i];
           }
           else if(fields[i].name == "z" && fields[i].datatype == pcl::PCLPointField::FLOAT32)
           {
              has_z = true;
              z_field_ = fields[i];
           }
           else if(fields[i].name == "intensity" && fields[i].datatype == pcl::PCLPointField::FLOAT32)
           {
              has_intensity_ = true;
              intensity_field_ = fields[i];
           }
           else if(fields[i].name == "ring" && fields[i].datatype == pcl::PCLPointField::UINT16)
           {
              has_ring_ = true;
              ring_field_ = fields[i];
           }
           else if(fields[i].name == "timestamp" && fields[i].datatype == pcl::PCLPointField::FLOAT64)
           {
              has_timestamp_ = true;
              timestamp_field_ = fields[i];
           }
           else if(fields[i].name == "yaw" && fields[i].datatype == pcl::PCLPointField::FLOAT32)
           {
              has_yaw_ = true;
              yaw_field_ = fields[i];
           }
       }
       (void) has_x;
       (void) has_y;
       (void) has_z;
       assert( has_x && has_y && has_z);
       check_point_type_ = true;
    }

    if ( scan.packets.size() == 0 )
    {
        return 0;
    }

    // get the total point number in this scan and initialize the point cloud based on this point number
    //pcd.points.reserve(scan.packets.size() * VELODYNE_POINTS_PER_PACKET);
    size_t packet_size = scan.packets.size();
    pcd.points.resize(packet_size * VELODYNE_POINTS_PER_PACKET);

    pcd.header.frame_id = scan.coord_id;

    Timestamp base_hour;
    base_hour.sec = 0;
    base_hour.nsec = 0;

    // check if has gps
    if (!(scan.timestamp.sec == 0 && scan.timestamp.nsec == 0))
    {
        base_hour = UtcToGps(scan.timestamp);
    }

    uint32_t last_velodyne_timestamp = scan.packets[0].velodyne_timestamp;

    size_t ptr = 0;
    for (size_t i = 0; i < scan.packets.size(); i++)
    {
        /// update the base hour and time if needed
        const VelodynePacket &packet = scan.packets[i];

        if ( !CheckPacketFrame(last_velodyne_timestamp, packet, base_hour) )
        {
            continue;
        }

        /* update the timestamp */
        last_velodyne_timestamp = packet.velodyne_timestamp;
        Timestamp base_time = Add(base_hour, Last(scan.packets[i].velodyne_timestamp/1e6));

        /* fill the pcd header with the first packet timestamp */
        if ( i == 0 )
        {
            stamp = (double)base_time.sec + 1e-9 * (double)base_time.nsec;
            //std::cout << std::setprecision(9) << "timestamp in pcd:" << base_time.sec << std::endl;
        }
        /* parse the packet and get the count */
        size_t count = UnpackVLP16(packet, convert_param_.grid, base_time, ptr, pcd);
        ptr += count;
    }

    pcd.width = scan.packets.size() * VELODYNE_BLOCKS_PER_PACKET * VLP16_SECTORS_PER_BLOCK;
    pcd.height = VLP16_SCANS_PER_SECTOR;

    // LOG(INFO) << "width size: " << pcd.width;
    // LOG(INFO) << "height size: " << pcd.height;
    // LOG(INFO) << "total size: " << pcd.width * pcd.height;

    return pcd.width * pcd.height;

}

size_t VelodynePcdConverter::UnpackVLP16(
            const VelodynePacket& packet,
            const bool grid,
            const Timestamp &base_time,
            const size_t offset,
            pcl::PointCloud<PointXYZIT> &pcd) const
{
    float32_t azimuths[VELODYNE_BLOCKS_PER_PACKET][VLP16_SECTORS_PER_BLOCK];
    GetPacketAzimuth(packet, azimuths);

    size_t count = 0;

    for (int block = 0; block < VELODYNE_BLOCKS_PER_PACKET; block++)
    {
        //check if the header is correct
        if ( !packet.blocks[block].Valid() )
        {
            LOG(ERROR) << "UnpackVLP16 meets invalid block header ";
            continue;
        }

        /* sector */
        for (int sector = 0, k = 0; sector < VLP16_SECTORS_PER_BLOCK; sector++)
        {
            const VelodyneBlock &b = packet.blocks[block];

            /* apply azimuth check if not dense */
            if ( !grid )
            {
                /* regular range, like [90,360] */
                if ( convert_param_.start_azimuth <= convert_param_.end_azimuth )
                {
                    if ( azimuths[block][sector] < convert_param_.start_azimuth || azimuths[block][sector] > convert_param_.end_azimuth )
                    {
                        continue;
                    }
                }
                /* irregular range, like [180, 90] */
                else
                {
                    if ( azimuths[block][sector] < convert_param_.start_azimuth && azimuths[block][sector] > convert_param_.end_azimuth )
                    {
                        continue;
                    }
                }
            }

            //for VLP16 each block has two sectors
            for (int ring = 0; ring < VLP16_SCANS_PER_SECTOR; ring++, k++)
            {
                const float distance = b.Depth(k);

                if ( !grid )
                {
                    if (distance < convert_param_.min_dis || distance > convert_param_.max_dis )
                    {
                        //the distance is out of range
                        continue;
                    }
                }

                float32_t theta = azimuths[block][sector] * M_PI / 180;
                float32_t x = distance * cos_vertical_vlp16_[ring] * std::sin(theta);
                float32_t y = distance * cos_vertical_vlp16_[ring] * std::cos(theta);
                float32_t z = distance * sin_vertical_vlp16_[ring];
                // putting point information to the pointcloud
                PointXYZIT &p = pcd.points.at(offset+count);
                pcl::setFieldValue<PointXYZIT, float32_t>(p, x_field_.offset, x);
                pcl::setFieldValue<PointXYZIT, float32_t>(p, y_field_.offset, y);
                pcl::setFieldValue<PointXYZIT, float32_t>(p, z_field_.offset, z);
                if(has_intensity_)
                {
                    int32_t intensity = b.Intensity(k);
                    pcl::setFieldValue<PointXYZIT, float32_t>(p, intensity_field_.offset, (float32_t)intensity);
                }
                if(has_ring_)
                {
                    pcl::setFieldValue<PointXYZIT, uint16_t>(p, ring_field_.offset, (uint16_t)ring);
                }
                if(has_yaw_)
                {
                    pcl::setFieldValue<PointXYZIT, float32_t>(p, yaw_field_.offset, (float32_t)theta);
                }
                if(has_timestamp_)
                {
                    Timestamp timestamp = Add(base_time, Last(((2 * block + sector) * VLP16_SECTOR_TOFFSET + ring * VLP16_DSR_TOFFSET)/1e6));
                    pcl::setFieldValue<PointXYZIT, float64_t>(p, timestamp_field_.offset, ToSec(timestamp));
                }

                count++;
            }
        }
    }

    return count;
}

}
}
}
