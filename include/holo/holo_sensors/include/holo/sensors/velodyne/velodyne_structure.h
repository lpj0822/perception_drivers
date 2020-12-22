/*!
* \brief This file defines velodyne raw info data struct for objects attribute
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef _HOLO_SENSORS_VELODYNE_STRUCTURES_H
#define _HOLO_SENSORS_VELODYNE_STRUCTURES_H

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
#include <holo/types.h>

namespace holo
{
namespace sensors
{
namespace velodyne
{
static const uint16_t VLP_HEADER = 0xeeff;

static const float32_t    VELODYNE_ROTATION_RESOLUTION = 0.01f; /**< degrees */
static const uint16_t VELODYNE_ROTATION_MAX_UNITS = 36000;

static const float32_t VELODYNE_DEPTH_MAX = 130.0f;        /**< meters */
static const float32_t VELODYNE_DEPTH_RESOLUTION = 0.002f; /**< meters */
static const int32_t   VELODYNE_DEPTH_MAX_UNITS = (VELODYNE_DEPTH_MAX /
        VELODYNE_DEPTH_RESOLUTION + 1);

static const int32_t VELODYNE_BLOCK_BYTE_SIZE = 100;
static const int32_t VELODYNE_RAW_POINT_BYTE_SIZE = 3;
static const int32_t VELODYNE_POINTS_PER_BLOCK = 32;
static const int32_t VELODYNE_BLOCK_DATA_BYTE_SIZE = (VELODYNE_POINTS_PER_BLOCK*
        VELODYNE_RAW_POINT_BYTE_SIZE);

static const int32_t VELODYNE_PACKET_BYTE_SIZE = 1206;
static const int32_t VELODYNE_BLOCKS_PER_PACKET = 12;
static const int32_t VELODYNE_PACKET_STATUS_SIZE = 2;
static const int32_t VELODYNE_PACKET_STAMPS_SIZE = 4;
static const int32_t VELODYNE_POINTS_PER_PACKET = (VELODYNE_POINTS_PER_BLOCK*
        VELODYNE_BLOCKS_PER_PACKET);

static const int32_t VLP16_SECTORS_PER_BLOCK = 2;
static const int32_t VLP16_SCANS_PER_SECTOR = 16;
static const float32_t VLP16_BLOCK_TDURATION = 110.592;
static const float32_t VLP16_DSR_TOFFSET = 2.304;
static const float32_t VLP16_SECTOR_TOFFSET = 55.296;

static const int32_t VELODYNE_GPS_PACKET_BYTE_SIZE = 512;
static const int32_t VELODYNE_NMEA_BYTE_SIZE = 72;

/* packet.reserved upper byte */
static const uint16_t VELODYNE_STRONGEST_RETURN = 0x37;
static const uint16_t VELODYNE_LAST_RETURN = 0x38;
static const uint16_t VELODYNE_DUAL_RETURN = 0x39;

/* packet.reserved lower byte */
static const uint16_t VELODYNE_HDL32 = 0x21;
static const uint16_t VELODYNE_VLP16 = 0x22;
static const uint16_t VELODYNE_VLP32A = 0x23;

static const float64_t VELODYNE_HDL32_PACKET_TIME =
    0.55296;  /* the time to receive one packet in ms */
static const float64_t VELODYNE_VLP16_PACKET_TIME =
    1.327104; /* the time to receive one packet in ms */

class VelodyneBlock
{
public:
    uint16_t header;   // eeff or ddff
    uint16_t rotation; // .01 degree
    /* The size of VelodyneMeasurement is 4 because of some reason, so do not use this class in block */
    /* VelodyneMeasurement data[POINTS_PER_BLOCK]; */
    uint8_t data[VELODYNE_BLOCK_DATA_BYTE_SIZE];

    bool Valid() const
    {
        return header == VLP_HEADER;
    }

    /* azimuth around the z-axis in degree */
    float32_t Azimuth() const
    {
        return rotation * VELODYNE_ROTATION_RESOLUTION;
    }

    /* depth of ith measurement in meter */
    float32_t Depth(size_t i) const
    {
        return (data[i * 3] /* lsb first */ + (data[i * 3 + 1] << 8)) *
               VELODYNE_DEPTH_RESOLUTION ;
    }

    /* intensity of ith measurement */
    uint8_t Intensity(size_t i) const
    {
        return data[i * 3 + 2] ;
    }


private:

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        (void) version;
        ar& header;
        ar& rotation;
        ar& boost::serialization::make_array(data, VELODYNE_BLOCK_DATA_BYTE_SIZE);
    }
};

class VelodynePacket
{
public:
    VelodyneBlock blocks[VELODYNE_BLOCKS_PER_PACKET];

    /* microseconds since top of the hour (synced w GPS every sec), Latched to first firing of the first firing sequence */
    uint32_t velodyne_timestamp;   /* the time stamp comes with the velodyne sensor, wrap around every hour */
    uint16_t reserved;             /* records the sensor type: vlp16, vlp32, and return type: strongest, closest, farest */

    /* for holding the data via recvfrom */
    void* Data()
    {
        return &blocks[0];
    }

    const VelodyneBlock& Block(size_t i) const
    {
        return blocks[i];
    }

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        (void) version;
        ar& boost::serialization::make_array(blocks, VELODYNE_BLOCKS_PER_PACKET);
        ar& velodyne_timestamp;
        ar& reserved;
    }
};

class VelodyneScan
{
public:
    std::string                       coord_id;        /* the source of the velodyne scan */
    Timestamp
    timestamp;    /* the timestamp at the moment of receiving a complete velodyne scan, but it is only the seconds outside the hour. the seconds within the hour can be found in the packet*/
    std::vector<VelodynePacket>
    packets;      /* due to the number of packets for one scan is not constant (cut_in angle), use vector here*/

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        (void) version;
        ar& coord_id;
        ar& timestamp;
        ar& packets;
    }
};

class VelodyneGpsRaw
{
public:
    uint8_t unused_1[198];
    uint8_t timestamp[4];
    uint8_t pps_state;                /* 0: absent, 1: attemp to sycn, 2: locked, 3: error */
    uint8_t unused_2[3];               
    uint8_t nmea[VELODYNE_NMEA_BYTE_SIZE];                 /* 0: "V" invalid, 1:"A" received, 2: unkonwn */
    uint8_t unused_3[234];            /* gps timestamp */

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        (void) version;
        ar& boost::serialization::make_array(unused_1, 198);
        ar& boost::serialization::make_array(timestamp, 4);
        ar& pps_state;
        ar& boost::serialization::make_array(unused_2, 3);
        ar& boost::serialization::make_array(nmea, VELODYNE_NMEA_BYTE_SIZE);
        ar& boost::serialization::make_array(unused_3, 234);
    }
};

class VelodyneSync
{
public:
     uint8_t   pps_state;
     uint8_t   nmea_state;
     uint32_t  gps_timestamp;

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        (void) version;
        ar& pps_state;
        ar& nmea_state;
        ar& gps_timestamp;
    }
};

}
}
}

#endif
