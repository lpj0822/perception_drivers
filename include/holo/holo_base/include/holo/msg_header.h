/*!
 *  \brief This file defines MsgHeader class
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLO_COMMON_MSG_HEADER_H_
#define HOLO_COMMON_MSG_HEADER_H_

#include <cmath>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <holo/types.h>
#include <glog/logging.h>

namespace holo
{

/**
 * \ingroup common
 * \brief holo message header class.
 *
 * Message header is prepended to messages that is being published.
 * Message header contains sequence, timestamp and coordinate field.\n
 * Sequence field is used for reorder published message at reception endpoint.\n
 * Timestamp field represents the time point the message is generated.\n
 * Coordinate field represents coordinate the value of message is referenced to.
 *
 */

static const uint32_t kUtcGpsGap = 315964800; ///< Time gap between UTC and GPS in seconds
static const uint32_t kNtpUtcGap = 2208988800; ///< Time Gap between NTP and GPS in seconds
static const uint32_t kGpsLeapSecond = 18; ///< GPS leap second

enum Coord
{
    WORLD = 0x0000,             ///< global world coordinate system
    BODY = 0x0100,              ///< vehicle body coordinate system

    NOVATEL = 0x0200,           ///< novatel coordinate system rotating 90 degrees clockwise from body coord
    UBLOX = 0x0201,             ///< UBLOX

    VLP_FRONT_LEFT = 0x0300,    ///< front_left velodyne coordinate system could be vlp16 or vlp32
    VLP_FRONT_RIGHT = 0x0301,   ///< front_right velodyne coordinate system could be vlp16 or vlp32
    VLP_REAR = 0x0302,          ///< rear velodyne coordinate system could be vlp16 or vlp32

    IBEO_FRONT_CENTER = 0x0310, ///< front center ibeo coordinate system could be lux or scala will be used if we do our pointcloud fusion
    IBEO_FRONT_LEFT = 0x0311,   ///< front left ibeo coordinate system could be lux or scala will be used if we do our pointcloud fusion
    IBEO_FRONT_RIGHT = 0x0312,  ///< front right ibeo coordinate system could be lux or scala will be used if we do our pointcloud fusion
    IBEO_REAR_LEFT = 0x0313,    ///< rear left ibeo coordinate system could be lux or scala will be used if we do our pointcloud fusion
    IBEO_REAR_RIGHT = 0x0314,   ///< rear right ibeo coordinate system could be lux or scala will be used if we do our pointcloud fusion
    IBEO_REAR_CENTER = 0x0315,  ///< rear center ibeo coordinate system could be lux or scala will be used if we do our pointcloud fusion
    IBEO_FUSION = 0x316,        ///< fusion box output of ibeo similar with body coordinate, but may have mesurement errors

    CONTI_FRONT_CENTER = 0x0400,
    CONTI_FRONT_LEFT = 0x0401,
    CONTI_FRONT_RIGHT = 0x0402,
    CONTI_REAR_LEFT = 0x0403,
    CONTI_REAR_RIGHT = 0x0404,
    CONTI_REAR_CENTER = 0x0405,

    DELPHI_ESR_FRONT_CENTER = 0x500,
    DELPHI_ESR_REAR_CENTER = 0x501,
    DELPHI_SRR3_FRONT_LEFT = 0x502,
    DELPHI_SRR3_FRONT_RIGHT = 0x503,
    DELPHI_SRR3_REAR_LEFT = 0x504,
    DELPHI_SRR3_REAR_RIGHT = 0x505,

    CAMERA_FRONT_CENTER = 0x0600, ///< front center camera coordinate system
    CAMERA_FRONT_LEFT = 0x0601,   ///< our own camera coordinate system
    CAMERA_FRONT_RIGHT = 0x0602,  ///< our own camera coordinate system
    CAMERA_REAR_CENTER = 0x0603,  ///< our own camera coordinate system

    UNKNOWN = 0xFFFF,          ///< not set yet, or unkonwn
};

struct Timestamp
{
    uint32_t sec;
    uint32_t nsec;
};

struct Duration
{
    int32_t sec;
    int32_t nsec;
};

static Timestamp Now()
{
    struct timeval tv;
    Timestamp ts;
    gettimeofday(&tv, NULL);
    ts.sec = tv.tv_sec;
    ts.nsec = tv.tv_usec * 1000;
    return ts;
}

static Timestamp UtcToGps(const Timestamp& utc)
{
    Timestamp ts;
    ts.sec = (uint64_t)utc.sec - (uint64_t)kUtcGpsGap + kGpsLeapSecond;
    ts.nsec = utc.nsec;
    return ts;
}

static Timestamp NowToGps()
{
    return UtcToGps(Now());
}

static Duration Last(float64_t t)
{
    Duration ts;
    int64_t sec64 = (int64_t)std::floor(t);
    if(sec64 < std::numeric_limits<int32_t>::min() ||
            sec64 > std::numeric_limits<int32_t>::max())
    {
        LOG(ERROR) << "Duration is out of 32-bit range";
    }
    ts.sec = (int32_t)sec64;
    ts.nsec = (int32_t)((t - ts.sec) * 1000000000);

    return ts;
}

static Duration NormalizeDuration(int64_t sec, int64_t nsec)
{
    sec += nsec / 1000000000;
    nsec %= 1000000000;
    if(nsec < 0)
    {
        nsec += 1000000000;
        sec--;
    }
    if(sec < std::numeric_limits<int32_t>::min() ||
            sec > std::numeric_limits<int32_t>::max())
    {
        //throw HoloException("Duration is out of 32-bit range");
    }

    Duration ts;
    ts.sec = sec;
    ts.nsec = nsec;

    return ts;
}

static Timestamp Normalize(int64_t sec, int64_t nsec)
{
    sec += nsec / 1000000000;
    nsec %= 1000000000;
    if(nsec < 0)
    {
        nsec += 1000000000;
        sec--;
    }
    if(sec < 0 || sec > std::numeric_limits<uint32_t>::max())
    {
        LOG(ERROR) << "Timestamp is out of 32-bit range";
    }

    Timestamp ts;
    ts.sec = uint32_t(sec);
    ts.nsec = uint32_t(nsec);

    return ts;
}

static Timestamp Add(const Timestamp lhs, const Duration& rhs)
{
    int64_t sec64 = (int64_t)lhs.sec + (int64_t)rhs.sec;
    int64_t nsec64 = (int64_t)lhs.nsec + (int64_t)rhs.nsec;
    Timestamp ts;
    ts = Normalize(sec64, nsec64);
    return ts;
}

static uint64_t ToNsec(uint32_t sec, uint32_t nsec)
{
    return (uint64_t)sec * 1000000000 + (uint64_t)nsec;
}

static float64_t ToSec(const Timestamp ts) 
{
    return (float64_t)ts.sec + 1e-9 * (float64_t)ts.nsec;
}


} //namespace holo

#endif
