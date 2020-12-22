/*!
* \brief This file defines pandar lidar scan to pointcloud converter
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#include <holo/sensors/pandar/pandar_pcd_converter.h>
#include <glog/logging.h>

namespace holo
{
namespace sensors
{
namespace pandar
{

PandarPcdConverter::PandarPcdConverter(const std::string& yaml_file)
{
    has_intensity_ = false;
    has_ring_ = false;
    has_timestamp_ = false;
    has_yaw_ = false;
    check_point_type_ = false;

    if(!LoadParameters(yaml_file))
    {
        return;
    }
}

bool PandarPcdConverter::LoadParameters(const std::string& yaml_file)
{
    try
    {
        yaml::Node root_node;
        root_node = yaml::LoadFile(yaml_file);
        convert_param_.calibration_file = root_node["calibration_file"].as<std::string>();
        convert_param_.time_correction_file = root_node["time_correction_file"].as<std::string>();
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
    catch(std::exception& e)
    {
        LOG(ERROR) << "Could not load yaml file " << yaml_file;
        return false;
    }

    try
    {
        yaml::Node correction_node;
        correction_node = yaml::LoadFile(convert_param_.calibration_file);
        for(size_t i=0; i != correction_node.size(); i++)
        {
            uint32_t laser_id = correction_node[i]["Laser_id"].as<uint32_t>() -1;
            if(i >= PANDAR_BEAMS_NUM || laser_id >= PANDAR_BEAMS_NUM)
            {
                LOG(ERROR) << "The correction parameters for Pandar is incorrect the index is " << i << " and laser id is " << laser_id;
                return false;
            }

            float32_t vertical_angle = (correction_node[i]["Vertical"].as<float32_t>()) / 180.0 * M_PI;
            float32_t azimuth_correction = correction_node[i]["Azimuth"].as<float32_t>();
            convert_param_.cos_vertical_angle[laser_id] = std::cos(vertical_angle);
            convert_param_.sin_vertical_angle[laser_id] = std::sin(vertical_angle);
            convert_param_.azimuth_correction[laser_id] = azimuth_correction;
        }

        yaml::Node time_correction_node;
        time_correction_node = yaml::LoadFile(convert_param_.time_correction_file);
        for(size_t i=0; i != time_correction_node.size(); i++)
        {
            uint32_t laser_id = time_correction_node[i]["Laser_id"].as<uint32_t>() -1;
            if(i >= PANDAR_BEAMS_NUM || laser_id >= PANDAR_BEAMS_NUM)
            {
                LOG(ERROR) << "The time correction parameters for Pandar is incorrect the index is " << i << " and laser id is " << laser_id;
                return false;
            }

            convert_param_.pt_time_offset_one[laser_id] = time_correction_node[i]["TimeOffset1"].as<uint16_t>();
            convert_param_.pt_time_offset_two[laser_id] = time_correction_node[i]["TimeOffset2"].as<uint16_t>();
            convert_param_.time_index[laser_id] = time_correction_node[i]["Index"].as<uint16_t>();
        }
    }
    catch(std::exception &e)
    {
        LOG(ERROR) << "Could not load correction parameters " << convert_param_.time_correction_file;
        return false;
    }

    return true;
}

size_t PandarPcdConverter::ParseScan(const PandarScan &scan, pcl::PointCloud<PointXYZIT> &pcd, double& stamp)
{
    if(!check_point_type_)
    {
       std::vector<pcl::PCLPointField> fields;
       pcl::getFields<PointXYZIT>(fields);
       bool has_x = false;
       bool has_y = false;
       bool has_z = false;

       for(size_t i = 0; i != fields.size(); i++)
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

    if (scan.packets.size() == 0 )
    {
        return 0;
    }

    // get the total point number in this scan and initialize the point cloud based on this point number
    //pcd.reserve(scan.packets.size() * PANDAR_POINTS_PER_PACKET);
    size_t packet_size = scan.packets.size();
    pcd.points.resize(packet_size * PANDAR_BLOCKS_PER_PACKET * PANDAR_BEAMS_NUM);

    pcd.header.frame_id = scan.coord;

    Timestamp base_hour;
    base_hour.sec = 0;
    base_hour.nsec = 0;

    // check if has gps
    if (!(scan.timestamp.sec == 0 && scan.timestamp.nsec == 0))
    {
        base_hour = UtcToGps(scan.timestamp) ;
    }

    size_t ptr = 0;
    
    for (size_t i = 0; i < scan.packets.size(); i++)
    {
        /// update the base hour and time if needed
        const PandarPacket &packet = scan.packets[i];

        /* update the timestamp */
        Timestamp base_time = Add(base_hour, Last(scan.packets[i].BlockTimeStamp()/1e6));
        //std::cout << std::setprecision(20) << "gps time in pcd:" << base_time.sec << std::endl;
        
        /* fill the pcd header with the first packet timestamp */
        if ( i == 0 )
        {
            stamp = (double)base_time.sec + 1e-9 * (double)base_time.nsec;
        }

        /* parse the packet and get the count */
        size_t count = UnpackPandar64(packet, convert_param_.grid, base_time, ptr, pcd);
        ptr += count;
    }

    pcd.width = scan.packets.size() * PANDAR_BLOCKS_PER_PACKET;
    pcd.height = PANDAR_BEAMS_NUM;

    // LOG(INFO) << "width size: " << pcd.width;
    // LOG(INFO) << "height size: " << pcd.height;
    // LOG(INFO) << "total size: " << pcd.width * pcd.height;

    return pcd.width * pcd.height;

}

size_t PandarPcdConverter::UnpackPandar64(
            const PandarPacket& packet,
            const bool grid,
            const Timestamp &base_time,
            const size_t offset,
            pcl::PointCloud<PointXYZIT> &pcd) const
{
    size_t count = 0;
    
    (void)grid;

      if(packet.header != PANDAR_HEADER)
      {
          LOG(ERROR) << "The header of block is incorrect " << packet.header;
      }

    for(size_t block_idx = 0; block_idx < PANDAR_BLOCKS_PER_PACKET; block_idx++)
    {
        PandarBlock block = packet.blocks[block_idx];

        float32_t base_azimuth = block.Azimuth();

        if(base_azimuth < convert_param_.start_azimuth || base_azimuth > convert_param_.end_azimuth)
        {
            continue;
        }

        float64_t block_offset_us = 0.0;
        /*single mode block time is block_idx(0-9) *55.56 ,dual mode block time is block_idx/2 *55.56*/
        if(packet.return_mode == PANDAR_DUAL_RETURN)
        {
            block_offset_us = PANDAR_BLOCK_TIME_OFFSET_US_1 + std::floor(block_idx/2) * PANDAR_BLOCK_TIME_OFFSET_US_2;
        }
        else
        {
            block_offset_us = PANDAR_BLOCK_TIME_OFFSET_US_1 + block_idx * PANDAR_BLOCK_TIME_OFFSET_US_2;
        }


        for(size_t pt_idx = 0; pt_idx < PANDAR_BEAMS_NUM; pt_idx++)
        {
            float32_t distance = block.Depth(pt_idx);
            uint32_t  intensity = block.Intensity(pt_idx);
            float32_t azimuth = base_azimuth + convert_param_.azimuth_correction[pt_idx];
            float32_t cos_azimuth = std::cos(azimuth / 180.0 * M_PI);
            float32_t sin_azimuth = std::sin(azimuth / 180.0 * M_PI);

            float x = distance * convert_param_.cos_vertical_angle[pt_idx] * sin_azimuth;
            float y = distance * convert_param_.cos_vertical_angle[pt_idx] * cos_azimuth;
            float z = distance * convert_param_.sin_vertical_angle[pt_idx];

            // putting point information to the pointcloud
            PointXYZIT &p  = pcd.points.at(offset+count+convert_param_.time_index[pt_idx]);

            pcl::setFieldValue<PointXYZIT, float>(p, x_field_.offset, x);
            pcl::setFieldValue<PointXYZIT, float>(p, y_field_.offset, y);
            pcl::setFieldValue<PointXYZIT, float>(p, z_field_.offset, z);

            if(has_intensity_)
            {
                pcl::setFieldValue<PointXYZIT, float32_t>(p, intensity_field_.offset, (float32_t)intensity);
            }
            if(has_ring_)
            {
                pcl::setFieldValue<PointXYZIT, uint16_t>(p, ring_field_.offset, (uint16_t)pt_idx);
            }
            if(has_yaw_)
            {
                pcl::setFieldValue<PointXYZIT, float32_t>(p, ring_field_.offset, (float32_t)azimuth);
            }
            if(has_timestamp_)
            {
                float64_t pt_offset_us = convert_param_.pt_time_offset_one[pt_idx] * PANDAR_PT_TIME_OFFSET_US_1 +
                                         convert_param_.pt_time_offset_two[pt_idx] * PANDAR_PT_TIME_OFFSET_US_2 + PANDAR_PT_TIME_OFFSET_US_3;
                Timestamp timestamp = Add(base_time, Last(-(pt_offset_us + block_offset_us)/1e6));
                pcl::setFieldValue<PointXYZIT, float64_t>(p, timestamp_field_.offset, ToSec(timestamp));
            }
        }
        count += PANDAR_BEAMS_NUM;
    }
    return count;
}

}
}
}
