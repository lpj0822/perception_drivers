/*!
* \brief This file defines novatel raw info data struct for objects attribute
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef _HOLO_NOVATEL_STRUCTURE_H
#define _HOLO_NOVATEL_STRUCTURE_H

#include <holo/types.h>

namespace holo
{
namespace sensors
{
namespace novatel
{

#define MAX_SIZE 8192                            // Maximum size of a NovAtel log buffer (ALMANACA logs are big!)
#define EPH_CHAN 33
#define NUMSAT 14
#define MAX_CHAN    54                           // Maximum number of signal channels
#define MAX_NUM_SAT 28                           // Maximum number of satellites with information in the RTKDATA log
#define HEADER_SIZE_NOVATEL 28                   // Binary header size for OEM 4, V, and 6 receivers
#define SHORT_HEADER_SIZE 12                     // short binary header size
#define CHECKSUM_SIZE 4                          // size of the message CRC
#define INSPVAX_MSG_SIZE 142                     // Binary header size for inspvax message

#define SYNC_1_IDX 0                             // first sync byte location
#define SYNC_2_IDX 1                             // second sync byte location
#define SYNC_3_IDX 2                             // third sync byte location
#define HEADER_LEN_IDX 3                         // header length location
#define MSG_ID_END_IDX 5                         // Message ID location
#define LENGTH_END_IDX 9                         // message length index

#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x44
#define SYNC_BYTE_3 0x12
#define SYNC_BYTE_3_SHORT 0x13
#define ACK_BYTE_1 '<'
#define ACK_BYTE_2 'O'
#define ACK_BYTE_3 'K'
#define RESET_BYTE_1 0x5B
#define RESET_BYTE_2 'C'
#define RESET_BYTE_3 'O'
#define RESET_BYTE_4 'M'
#define RESET_BYTE_6 0x5D

#define ICOM1_PORT  3001
#define ICOM2_PORT  3002
#define ICOM3_PORT  3003

#define SERIAL_BUFFER_SIZE  2048

// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

enum BINARY_LOG_TYPE
{
    GPSEPHEMB_LOG_TYPE = 7,
    IONUTCB_LOG_TYPE = 8 ,
    CLOCKMODELB_LOG_TYPE = 16,
    VERSIONB_LOG_TYPE = 37,
    RAWEPHEMB_LOG_TYPE = 41,
    BESTPOSB_LOG_TYPE = 42,
    BESTUTMB_LOG_TYPE = 726,
    BESTXYZB_LOG_TYPE = 241,
    RANGEB_LOG_TYPE = 43,
    PSRPOSB_LOG_TYPE = 47,
    SATVISB_LOG_TYPE = 48,
    ALMANACB_LOG_TYPE = 73,
    RAWALMB_LOG_TYPE = 74,
    TRACKSTATB_LOG_TYPE = 83,
    SATSTATB_LOG_TYPE = 84,
    SATXYZB_LOG_TYPE = 270,
    RXSTATUSB_LOG_TYPE = 93,
    RXSTATUSEVENTB_LOG_TYPE = 94,
    RXHWLEVELSB_LOG_TYPE = 195,
    MATCHEDPOSB_LOG_TYPE = 96,
    BESTVELB_LOG_TYPE = 99,
    PSRVELB_LOG_TYPE = 100,
    TIMEB_LOG_TYPE = 101,
    RANGEPNB_LOG_TYPE = 126,
    RXCONFIGB_LOG_TYPE = 128,
    RANGECMPB_LOG_TYPE = 140,
    RTKPOSB_LOG_TYPE = 141,
    RTKDOPB_LOG_TYPE = 952,
    NAVIGATEB_LOG_TYPE = 161,
    AVEPOSB_LOG_TYPE = 172,
    REFSTATIONB_LOG_TYPE = 175,
    PASSCOM1B_LOG_TYPE = 233,
    PASSCOM2B_LOG_TYPE = 234,
    PASSCOM3B_LOG_TYPE = 235,
    BSLNXYZ_LOG_TYPE = 686,
    PSRXYZ_LOG_TYPE = 243,
    PSRDOPB_LOG_TYPE = 174,
    BESTGNSSPOS_LOG_TYPE = 1429,
    BESTGNSSVEL_LOG_TYPE = 1430,

    //SPAN - INS specific logs
    BESTGPSPOS_LOG_TYPE = 423,
    BESTGPSVEL_LOG_TYPE = 506,
    BESTLEVERARM_LOG_TYPE = 674,
    INSATT_LOG_TYPE = 263,    // INS ATTITUDE
    INSCOV_LOG_TYPE = 264,    // INS covariance, each of PVA has a 3 by 3 covariance matrix
    INSCOVS_LOG_TYPE = 320,   // INSCOV with short header
    INSPOS_LOG_TYPE = 265,
    INSPOSSYNC_LOG_TYPE = 322,
    INSPVA_LOG_TYPE = 507,    // INS POSITION, VELOCITY, AND ATTITUDE
    INSPVAS_LOG_TYPE = 508,   // INS POSITION, VELOCITY, AND ATTITUDE short header
    INSPVAX_LOG_TYPE = 1465,  // INSPVA + standard deviation of PVA
    INSSPD_LOG_TYPE = 266,
    INSUTM_LOG_TYPE = 756,
    INSUPDATE_LOG_TYPE = 757,
    INSVEL_LOG_TYPE = 267,
    RAWIMU_LOG_TYPE = 268,    // RAWIMU, angular velocity and linear acceleration
    RAWIMUS_LOG_TYPE = 325,   // RAWIMU, but with short header
    RAWIMUSX_LOG_TYPE = 1462,  // RAWIMUS, but with extended status
    VEHICLEBODYROTATION_LOG_TYPE = 642,

    INSPOSX_LOG_TYPE = 1459,
    INSATTX_LOG_TYPE = 1457
};
typedef enum BINARY_LOG_TYPE BINARY_LOG_TYPE;

/// header for messages
PACK(
    struct Oem7Header
{
    uint8_t          sync1;          //!< start of packet first byte (0xAA)
    uint8_t          sync2;          //!< start of packet second byte (0x44)
    uint8_t          sync3;          //!< start of packet third  byte (0x12)
    uint8_t          header_length;  //!< Length of the header in bytes ( From start of packet )
    uint16_t         message_id;     //!< Message ID number
    uint8_t          message_type;   //!< Message type - binary, ascii, nmea, etc...
    uint8_t          port_address;   //!< Address of the data port the log was received on
    uint16_t         message_length; //!< Message length (Not including header or CRC)
    uint16_t         sequence;       //!< Counts down from N-1 to 0 for multiple related logs
    uint8_t          idle_time;      //!< Time the processor was idle in last sec between logs with same ID
    uint8_t          time_status;    //!< Indicates the quality of the GPS time (check TIME_STATUS for detail)
    uint16_t         gps_week;       //!< GPS Week number
    uint32_t         gps_millisecs;  //!< Milliseconds into week
    uint32_t         receiver_status;//!< Receiver status word
    uint16_t         reserved;       //!< Reserved for internal use
    uint16_t         software_version;//!< Receiver software build number (0-65535)
});

// message inspva combined gnss and imu results
PACK(
    struct InsPositionVelocityAttitudeExtended
{
    Oem7Header header;             //!< Message header
    uint32_t ins_status;           //!< Solution status
    uint32_t position_type;        //!< Position type 
    float64_t latitude;            //!< latitude (deg)
    float64_t longitude;           //!< longitude (deg)
    float64_t height;              //!< height above mean sea level (m)
    float32_t undulation;          //!< Undulation - the relationship between the geoid and the ellipsoid (m) 
    float64_t north_velocity;      //!< velocity in a northerly direction (m/s)
    float64_t east_velocity;       //!< velocity in an easterly direction (m/s)
    float64_t up_velocity;         //!< velocity in an up direction
    float64_t roll;                //!< right handed rotation around y-axis (degrees)
    float64_t pitch;               //!< right handed rotation aruond x-axis (degrees)
    float64_t azimuth;             //!< right handed rotation around z-axis (degrees)
    float32_t latitude_std;        //!< latitude standard deviation (m)
    float32_t longitude_std;       //!< longitude standard deviation (m)
    float32_t height_std;          //!< height standard deviation (m)
    float32_t north_velocity_std;  //!< north velocity standard deviation (m/s)
    float32_t east_velocity_std;   //!< east velocity standard deviation (m/s)
    float32_t up_velocity_std;     //!< up velocity standard deviation (m/s)
    float32_t roll_std;            //!< up velocity standard deviation (m/s)
    float32_t pitch_std;           //!< up velocity standard deviation (m/s)
    float32_t azimuth_std;         //!< up velocity standard deviation (m/s)
    uint32_t ext_solution_status;   //!< Extended solution status
    uint16_t time_since_update;    //!< Elapsed time since the last ZUPT or position update (s)
    uint32_t status;            //!< status of the INS system
    int8_t crc[4];                 //!< 32-bit checksum
});

}
}
}

#endif
