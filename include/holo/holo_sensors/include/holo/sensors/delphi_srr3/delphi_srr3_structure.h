/*!
* \brief This file defines delphi srr3 raw info data struct for objects attribute
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLO_SENSORS_DELPHI_SRR3_STRUCTURE_H_
#define HOLO_SENSORS_DELPHI_SRR3_STRUCTURE_H_

namespace holo
{
namespace sensors
{
namespace delphi_srr3
{

//! delphi-srr3 number in a single set
#define SRR3_SCANNERS_NUM       2
#define SRR3_SCANNERS_LEFT      0
#define SRR3_SCANNERS_RIGHT     1

//! delphi-srr3 system objects number in a single cycle
#define SRR3_OBJECTS_NUM        15

/**
 * @brief delphi srr3 object detection descriptor struct
 */
struct DelphiSrr3Object
{
    Timestamp timestamp;
    uint8_t obj_id;
    uint8_t object_motion_pattern;
    uint8_t object_type;
    uint8_t alive_time;
    uint8_t confidence;
    float32_t longitudinal_position;
    float32_t lateral_position;
    float32_t longitudinal_velocity;
    float32_t lateral_velocity;
    float32_t longitudinal_acceleration;
    float32_t lateral_acceleration;
    float32_t heading;
    float32_t width;
    bool is_coasted;
};

/**
 * @brief      detection list definition
 */
struct DelphiSrr3ObjectList
{
    //! timestamp
    Timestamp timestamp;
    //! coord
    enum Coord coord_id;
    //! objects
    DelphiSrr3Object objects[SRR3_OBJECTS_NUM];
};

//! @brief delphi_srr3 location description 
enum DelphiSrr3Location
{
    FRONT_CORNER,
    REAR_CORNER,
};

}   // namespace delphi_srr3
}   // namespace sensors
}   // namespace holo

#endif