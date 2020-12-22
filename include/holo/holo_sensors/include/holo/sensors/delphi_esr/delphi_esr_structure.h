/*!
* \brief This file defines delphi esr raw info data struct for objects attribute
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLO_SENSORS_DELPHI_ESR_STRUCTURE_H_
#define HOLO_SENSORS_DELPHI_ESR_STRUCTURE_H_

namespace holo
{
namespace sensors
{
namespace delphi_esr
{

//! delphi-esr sensor number
#define ESR_NUM                2
//! delphi-esr system objects number in a single cycle
#define ESR_OBJECTS_NUM        64

/**
 * @brief delphi esr object detection descriptor struct
 */
struct DelphiEsrObject
{
    Timestamp timestamp;
    uint8_t obj_id;
    uint8_t med_range_mode;
    uint8_t status;
    float32_t angle;
    float32_t range;
    float32_t range_accel;
    float32_t range_rate;
    float32_t width;     
    float32_t lat_rate;
    bool is_bridge;
    bool is_oncoming;
    bool is_grouping_changed;      
};

/**
 * @brief      detection list definition
 */
struct DelphiEsrObjectList
{
    //! timestamp
    Timestamp timestamp;
    //! coord
    enum Coord coord_id;
    //! objects
    DelphiEsrObject objects[ESR_OBJECTS_NUM];
};

//! @brief delphi_esr location description 
enum DelphiEsrLocation
{
    FRONT_CENTER,
    REAR_CENTER,
};

}   // namespace delphi_esr
}   // namespace sensors
}   // namespace holo

#endif