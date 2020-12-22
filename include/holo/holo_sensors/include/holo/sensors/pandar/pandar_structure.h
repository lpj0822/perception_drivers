/*!
* \brief This file defines pandar 64 raw info data struct for objects attribute
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef _HOLO_SENSORS_PANDAR_STRUCTURES_H
#define _HOLO_SENSORS_PANDAR_STRUCTURES_H

#include <arpa/inet.h>
#include <iosfwd>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/serialization/array.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <holo/msg_header.h>
#include <holo/point_cloud.h>

namespace holo
{
namespace sensors
{
namespace pandar
{

static const uint16_t PANDAR_HEADER = 0xFFEE;
static const float32_t PANDAR_ROTATION_RESOLUTION = 0.01f;

static const float32_t PANDAR_DEPTH_RESOLUTION = 0.004f;
static const uint16_t PANDAR_BEAMS_NUM = 64;
static const uint16_t PANDAR_POINT_SIZE = 3;
static const uint16_t PANDAR_BLOCK_DATA_BYTE_SIZE = PANDAR_BEAMS_NUM * PANDAR_POINT_SIZE;
static const uint16_t PANDAR_BLOCKS_PER_PACKET = 6;
static const uint16_t PANDAR_PACKET_BYTE_SIZE = 1194;
static const uint16_t PANDAR_GPS_PACKET_SIZE  = 512;

static const uint16_t PANDAR_POINTS_PER_PACKET = PANDAR_BLOCKS_PER_PACKET * PANDAR_BEAMS_NUM;
static const float32_t PANDAR_PT_TIME_OFFSET_US_1 = 1.304;
static const float32_t PANDAR_PT_TIME_OFFSET_US_2 = 1.968;
static const float32_t PANDAR_PT_TIME_OFFSET_US_3 = 3.62;
static const float32_t PANDAR_BLOCK_TIME_OFFSET_US_1 = 42.58;
static const float32_t PANDAR_BLOCK_TIME_OFFSET_US_2 = 55.56;

static const uint8_t PANDAR_STRONGEST_RETURN = 0x37;
static const uint8_t PANDAR_LAST_RETURN = 0x38;
static const uint8_t PANDAR_DUAL_RETURN = 0x39;

class PandarBlock
{
public:
	uint16_t rotation;
	uint8_t  data[PANDAR_BLOCK_DATA_BYTE_SIZE];

    float32_t Azimuth() const
    {
        return rotation * PANDAR_ROTATION_RESOLUTION;
    }

    float32_t Depth(size_t i) const
    {
        return (data[i * 3] + (data[i * 3 + 1] << 8)) *
               PANDAR_DEPTH_RESOLUTION;
    }

    uint32_t Intensity(size_t i) const
    {
        return data[i * 3 + 2];
    }

private:
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        (void)version;
        ar& rotation;
        ar& boost::serialization::make_array(data,
                                             PANDAR_BLOCK_DATA_BYTE_SIZE);
    }
};

class PandarPacket
{
public:
    uint16_t header;
    uint8_t laser_num;
    uint8_t block_num;
    uint8_t return_type;
    uint8_t distance_unit;
    uint8_t reserved1;
    uint8_t reserved2;
    PandarBlock blocks[PANDAR_BLOCKS_PER_PACKET];
    uint8_t reserved3[5];
    uint8_t high_temperature;
    uint16_t reserved4;
    uint16_t motor_speed;
    uint8_t block_timestamp[4];
    uint8_t return_mode;
    uint8_t factory_info;
    uint8_t utc_time[6];

    void* Data()
    {
        return &header;
    }

    const PandarBlock& Block(size_t i) const
    {
        return blocks[i];
    }

    uint32_t BlockTimeStamp() const
    {
        uint32_t timestamp_u32 = ((block_timestamp[3] << 24) |
                                  (block_timestamp[2] << 16) |
                                  (block_timestamp[1] << 8 ) |
                                  (block_timestamp[0]));
        return timestamp_u32;
    }
    void BlockTimeStamp(const uint32_t& _block_timestamp)
    {
        block_timestamp[3] = (_block_timestamp >> 24) & 0xFF;
        block_timestamp[2] = (_block_timestamp >> 16) & 0xFF;
        block_timestamp[1] = (_block_timestamp >> 8) & 0xFF;
        block_timestamp[0] = _block_timestamp & 0xFF;
    }

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        (void) version;
        ar& laser_num;
        ar& block_num;
        ar& return_type;
        ar& distance_unit;
        ar& reserved1;
        ar& reserved2;
        ar& boost::serialization::make_array(blocks, PANDAR_BLOCKS_PER_PACKET);
        ar& boost::serialization::make_array(reserved3, 5);
        ar& high_temperature;
        ar& reserved4;
        ar& motor_speed;
        ar& boost::serialization::make_array(block_timestamp, 4);
        ar& return_mode;
        ar& factory_info;
        ar& boost::serialization::make_array(utc_time, 6);
    }
};

class PandarScan
{
public:
	std::string    coord;
	Timestamp timestamp;
	std::vector<PandarPacket> packets;
};

class PandarGpsPacket
{
public:
	uint16_t header;
	uint8_t  date[6];
	uint8_t  time[6];
	uint8_t  us_time[4];
	uint8_t  gprmc[77];
	uint8_t  reserved1[411];
	uint8_t  location_valid;
	uint8_t  pps_locked;
	uint32_t reserved2;

private:
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        (void) version;
        ar& header;
        ar& boost::serialization::make_array(date, 6);
        ar& boost::serialization::make_array(time, 6);
        ar& boost::serialization::make_array(us_time,4);
        ar& boost::serialization::make_array(gprmc, 77);
        ar& boost::serialization::make_array(reserved1, 411);
        ar& location_valid;
        ar& pps_locked;
        ar& reserved2;
    }
};

}
}
}

#endif
