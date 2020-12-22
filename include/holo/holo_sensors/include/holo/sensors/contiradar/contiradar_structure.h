/*!
* \brief This file defines contiradar raw info data struct for objects attribute
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLO_SENSORS_CONTIRADAR_SIGNAL_H_
#define HOLO_SENSORS_CONTIRADAR_SIGNAL_H_

#include <holo/msg_header.h>

namespace holo
{
namespace sensors
{
namespace contiradar
{

//! @brief contiradar system objects maxium number in a single cycle
#define CONTIRADAR_OBJECTS_NUM    255

//! @brief how many radars installed on the vehicle
#define CONTIRADAR_NUM            6

#define CenterFront_SRR         6
#define CornerLeft_SRR          7
#define CornerRight_SRR         8
#define RearLeft_SRR            9
#define RearRight_SRR           10
#define CenterRear_SRR          11
#define Radar_Reserved          12


//! @brief contiradar location description 
enum Radar_location
{
    FRONT,
    CORNER_LEFT,
    CORNER_RIGHT,
    REAR_LEFT,
    REAR_RIGHT,
    REAR,
};


struct Ars048Obstacle
{
    Timestamp timestamp;
    enum Coord coord;

    //! @brief object ID
    //! @note since objects are tracked, the ID is kept throughout measurement cycles and does not have to be consecutive
    //! @par range
    //!      0..255
    //! @par precision
    //!      1
    //! @par unit
    //!      none
    uint8_t obj_id;
    
    //! @brief longitudinal coordinate
    //! @note the longitudinal distance between object and conti radar in the vehicle headway
    //! @par range
    //!      -500..1138.2
    //! @par precision
    //!      0.2
    //! @par unit
    //!      m
    float32_t obj_dist_long;

    //! @brief lateral coordinate
    //! @note the lateral distance between object and conti radar in the vehicle headway, positive to the left
    //! @par range
    //!      -204.6..204.8
    //! @par precision
    //!      0.2
    //! @par unit
    //!      m
    float32_t obj_dist_lat;

    //! @brief relative velocity in longitudinal direction
    //! @note the relative velocity to the object in longitudinal direction, positive for receding, negative for closing in 
    //! @par range
    //!      -128.00..127.75
    //! @par precision
    //!      0.25
    //! @par unit
    //!      m/s
    float32_t obj_vrel_long;


    //! @brief relative velocity in lateral direction
    //! @note the relative velocity to the object in lateral direction, positive for receding, negative for closing in
    //! @par range
    //!      -64.00..63.75
    //! @par precision
    //!      0.25
    //! @par unit
    //!      m/s
    float32_t obj_vrel_lat;

    //! @brief dynamic property of the object
    //! @note indicating if the object is moving or stationary (this value can only be determined correctly if the speed and yaw rate is given correctly)
    //! @par range
    //!      0  moving
    //!      1  stationary
    //!      2  oncoming
    //!      3  stationary candidate
    //!      4  unknown
    //!      5  crossing stationary
    //!      6  crossing moving
        //!      7  stopped
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    uint8_t   obj_dyn_prop;

    //! @brief radar cross section
        //! @note measure how detectable an object is with a radar. a larger RCS indicates that an object is more easily deteced 
        //! @par range
        //!      -64.0..63.5
        //! @par precision
        //!      0.5
        //! @par unit
        //!      dBm^2
    float32_t obj_rcs;

//! @brief standard deviation of longitudinal distance
        //! @note the standard deviation of longitudinal distance
        //! @par range
        //!      0..31
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    float32_t obj_dist_long_rms;

//! @brief standard deviation of lateral distance
        //! @note the standard deviation of lateral distance
        //! @par range
        //!      0..31
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    float32_t obj_dist_lat_rms;

    //! @brief standard deviation of longitudinal relative velocity
        //! @note the standard deviation of longitudinal relative velocity
        //! @par range
        //!      0..31
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    float32_t obj_vrel_long_rms;

//! @brief standard deviation of lateral relative velocity
        //! @note the standard deviation of lateral relative velocity
        //! @par range
        //!      0..31
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    float32_t obj_vrel_lat_rms;

    //! @brief standard deviation of longitudinal relative acceleration
        //! @note the standard deviation of longitudinal relative acceleration
        //! @par range
        //!      0..31
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    float32_t obj_arel_long_rms;

    //! @brief standard deviation of lateral relative acceleration
        //! @note the standard deviation of lateral relative acceleration
        //! @par range
        //!      0..31
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    float32_t obj_arel_lat_rms;

    //! @brief standard deviation of orientation angle
        //! @note the standard deviation of orientation angle
        //! @par range
        //!      0..31
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    float32_t obj_orientation_rms;

    //! @brief measurement state
        //! @note indicating if the object is valid and has been confirmed by clusters in the new measurement cycle
        //! @par range
        //!      0  deleted
        //!      1  new
        //!      2  measured
        //!      3  predicted
        //!      4  deleted for 
        //!      5  new from merge
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    uint8_t obj_meas_state;

    //! @brief probability of existance
        //! @note the probability of existance of an detected object
        //! @par range
        //!      0  invalid
        //!      1  < 25%
        //!      2  < 50%
        //!      3  < 75%
        //!      4  < 90% 
        //!      5  < 99%
        //!      6  < 99.9% 
        //!      7  <= 100%        
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    uint8_t obj_prob_of_exist;

    //! @brief relative acceleration in longitudinal direction
        //! @note the relative acceleration to the object in longitudinal direction, positive means the acceleration of object is higher than ego vehicle, 
        //!       negative means the acceleration of object is lower than ego vehicle 
        //! @par range
        //!      -10.00..10.47
        //! @par precision
        //!      0.01
        //! @par unit
        //!      m/s^2
    float32_t obj_arel_long;

    //! @brief relative acceleration in lateral direction
        //! @note the relative acceleration to the object in lateral direction, positive means the acceleration of object is higher than ego vehicle, 
        //!       negative means the acceleration of object is lower than ego vehicle 
        //! @par range
        //!      -2.50..2.61
        //! @par precision
        //!      0.01
        //! @par unit
        //!      m/s^2
    float32_t obj_arel_lat;

    //! @brief object class
        //! @note the classification of different object types
        //! @par range
        //!      0  point
        //!      1  car
        //!      2  truck
        //!      3  pedestrian
        //!      4  motorcycle
        //!      5  bicycle
        //!      6  wide 
        //!      7  reserved 
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    uint8_t obj_class;

    //! @brief orientation angle of the object
        //! @note positive means the object is in left side of ego vehicle while negative means the object is in right side of ego vehicle  
        //! @par range
        //!      -180.00..180.00
        //! @par precision
        //!      0.4
        //! @par unit
        //!      deg
    float32_t obj_orientation_angle;

    //! @brief length of the tracked object
        //! @note length of the tracked object
        //! @par range
        //!      0..51.0
        //! @par precision
        //!      0.2
        //! @par unit
        //!      m
    float32_t obj_length;

    //! @brief width of the tracked object
        //! @note width of the tracked object
        //! @par range
        //!      0..51.0
        //! @par precision
        //!      0.2
        //! @par unit
        //!      m
    float32_t obj_width;

    //! @brief false alarm probability of cluster
        //! @note false alarm probability of cluster
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    uint8_t pdh0;    

    //! @brief state of Doppler ambiguity solution of cluster
        //! @note state of Doppler ambiguity solution of cluster
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    uint8_t ambig_state; 

    //! @brief state of cluster validity
        //! @note state of cluster validity
        //! @par precision
        //!      1
        //! @par unit
        //!      none
    uint8_t invalid_state;         
};

struct Ars048ObstacleList
{
    Timestamp timestamp;
    enum Coord coord;
    std::vector<struct Ars048Obstacle> obstacle_list;
};

//! @brief contiradar cluster raw info data struct
//! Contains a standard header for frame_id and timestamp, and partX_valid for
//! indicates if the partX substruct data is valid. These part1~2 correspond to
//! these raw can messages from conti radar.
struct ContiRadarCluster
{
    Timestamp timestamp;
    enum Coord coord;

    //! status valid indicator
    bool status_valid;
    //! status data
    struct
    {
        //! @brief number of near clusters
        //! @note the number of near clusters that conti radar detects
        //! @par range
        //!      0..255
        //! @par precision
        //!      1
        //! @par uint 
        //!      none
        uint8_t NofClustersNear;

        //! @brief number of far clusters
        //! @note the number of far clusters that conti radar detects
        //! @par range
        //!      0..255
        //! @par precision
        //!      1
        //! @par uint 
        //!      none
        uint8_t NofClustersFar;

        //! @brief measurement cycle counter
        //! @note counting up since startup of sensor and restarting at 0 when > 65535 
        //! @par range
        //!      0..65535

        //! @par precision
        //!      1
        //! @par unit
        //!      none
        uint32_t MeasCounter;

        //! @brief object list CAN interface version
        //! @note object list CAN interface version
        //! @par range
        //!      0..15
        //! @par precision
        //!      1
        //! @par unit
        //!      none
        uint32_t InterfaceVersion;
    } status;

    //! part1 data valid indicator
    bool part1_valid;
    //! part1 data
    struct
    {
        uint32_t ID;

        
        uint32_t DistLong;

        
        uint32_t DistLat;

        
        uint32_t VrelLong;

        
        uint32_t VrelLat;

        
        uint32_t DynProp;

        
        uint32_t RCS;
    } part1;

    //! part2 data valid indicator
    bool part2_valid;
    //! part2 data
    struct
    {
        
        uint32_t ID;

        
        uint32_t DistLong_rms;

        
        uint32_t VrelLong_rms;

        
        uint32_t DistLat_rms;

        
        uint32_t VrelLat_rms;

        
        uint8_t Pdh0;

        
        uint8_t AmbigState;

        
        uint8_t InvalidState;        
    } part2;
};

//! @brief contiradar object raw info data struct
//! @note please note that the members of part1~3 has names that NOT valid
//!       according to holo coding style. this is reasonable because in
//!       THIS file these names are also used as string when they are provided
//!       as params of some local macros which simply the code.
//! 
//! Contains a standard header for frame_id and timestamp, and partX_valid for
//! indicates if the partX substruct data is valid. These part1~3 correspond to
//! these raw can messages from conti radar.
struct ContiRadarObject
{
    Timestamp timestamp;
    enum Coord coord;

    //! status valid indicator
    bool status_valid;
    //! status data
    struct
    {
        //! @brief number of objects
        //! @note the number of objects that conti radar detects
        //! @par range
        //!      0..255
        //! @par precision
        //!      1
        //! @par uint 
        //!      none
        uint32_t NofObjects;

        //! @brief measurement cycle counter
        //! @note counting up since startup of sensor and restarting at 0 when > 65535 
        //! @par range
        //!      0..65535

        //! @par precision
        //!      1
        //! @par unit
        //!      none
        uint32_t MeasCounter;

        //! @brief object list CAN interface version
        //! @note object list CAN interface version
        //! @par range
        //!      0..15
        //! @par precision
        //!      1
        //! @par unit
        //!      none
        uint32_t InterfaceVersion;
    } status;

    //! part1 data valid indicator
    bool part1_valid;
    //! part1 data
    struct
    {
        uint32_t ID;

        
        uint32_t DistLong;

        
        uint32_t DistLat;

        
        uint32_t VrelLong;

        
        uint32_t VrelLat;

        
        uint32_t DynProp;

        
        uint32_t RCS;
    } part1;

    //! part2 data valid indicator
    bool part2_valid;
    //! part2 data
    struct
    {
        
        uint32_t ID;

        
        uint32_t DistLong_rms;

        
        uint32_t VrelLong_rms;

        
        uint32_t ArelLong_rms;

        
        uint32_t DistLat_rms;

        
        uint32_t VrelLat_rms;

        
        uint32_t ArelLat_rms;

        
        uint32_t Orientation_rms;

        
        uint32_t MeasState;

        
        uint32_t ProbOfexist;        
    } part2;

    //! part3 data valid indicator
    bool part3_valid;
    //! part3 data
    struct
    {
        uint32_t ID;

        
        uint32_t ArelLong;

        
        uint32_t Class;

        
        uint32_t ArelLat;

        
        uint32_t OrientationAngle;

        
        uint32_t Length;

        
        uint32_t Width;        
    } part3;
};

struct Config_SendCommand
{
    uint8_t header[4];
    uint8_t radar_location;
    uint8_t output_type;
    uint8_t send_quality;
    uint8_t send_extend;
    uint32_t max_distance;
    uint8_t radar_power;
    uint8_t rcs_threshold;
    uint8_t filter_index;
    uint8_t filter_type;
    float32_t filter_min_value;
    float32_t filter_max_value;
};

}   // namespace contiradar
}   // namespace sensors
}   // namespace holo

#endif
