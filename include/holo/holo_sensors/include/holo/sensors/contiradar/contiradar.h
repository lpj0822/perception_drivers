/*!
* \brief This file defines ContiRadar class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLO_SENSORS_CONTIRADAR_H_
#define HOLO_SENSORS_CONTIRADAR_H_

#include <stdio.h>
#include <unistd.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <holo/msg_header.h>
#include <holo/types.h>
#include <holo/yaml.h>

#include <holo/sensors/candev/candev.h>
#include <holo/sensors/contiradar/contiradar_obj.h>
#include <holo/sensors/contiradar/contiradar_cluster.h>
#include <holo/sensors/contiradar/contiradar_structure.h>

namespace holo
{
namespace sensors
{
namespace contiradar
{

//! @brief defines a class for contiradar data analysing and transferring
class Contiradar
{

//! ubyte for unsigned 8-bits integer
typedef unsigned char ubyte;
//! uword for unsigned 16-bits integer
typedef unsigned short uword;
//! ulong for unsigned 32-bits integer
typedef unsigned int ulong;

// Callback
typedef std::function<void(const Ars048ObstacleList&)> Ars048ObstaclesCallback;

public:
    //! contiradar driver constructor
    Contiradar(const std::string& yaml_file);

    void InstallServiceRawObsCallback(const Ars048ObstaclesCallback& callback, int radar_index);

private:
    // //! total number of objects received in one complete cycle

    int32_t number_of_objects_[CONTIRADAR_NUM];

    int32_t index_part1_[CONTIRADAR_NUM];

    int32_t index_part2_[CONTIRADAR_NUM];

    int32_t index_part3_[CONTIRADAR_NUM];

    bool first_time_object_[CONTIRADAR_NUM];

    bool first_time_object_part1_[CONTIRADAR_NUM];

    ContiRadarObject local_object_[CONTIRADAR_NUM][CONTIRADAR_OBJECTS_NUM];

    ContiRadarObject temp_object_[CONTIRADAR_NUM];

    ContiRadarCluster local_cluster_[CONTIRADAR_NUM][CONTIRADAR_OBJECTS_NUM];

    ContiRadarCluster temp_cluster_[CONTIRADAR_NUM];

    Ars048ObstacleList objects_[CONTIRADAR_NUM];

    Ars048ObstacleList converted_objects_[CONTIRADAR_NUM];

    Ars048ObstaclesCallback ars048_obstacles_callback_[CONTIRADAR_NUM];

    Ars048ObstaclesCallback ars048_convert_callback_[CONTIRADAR_NUM];

    int32_t sock_;
    struct sockaddr_in addr_;

    //! the mounting location for contiradar
    std::string radar_location_;
    //! socket type of can communication, like 'tcp'/'udp'
    std::string socket_type_;
    //! local host server ip, like '192.168.1.3'
    std::string host_ip_;
    //! local host server port used, like '5021'
    std::string host_port_;
    //! remote client ip, like '192.168.1.101'
    std::string client_ip_;
    //! remote client port used, like '5000'
    std::string client_port_;

    struct Config_SendCommand send_command;

    // radar config parameters
    std::string output_type_;
    uint8_t send_quality_;
    uint8_t send_extend_;
    uint32_t max_distance_;
    uint8_t radar_power_;
    uint8_t rcs_threshold_;

    // radar filter config parameters
    uint8_t filter_index_;
    std::string filter_type_;
    float32_t filter_min_value_;
    float32_t filter_max_value_;   

    //! can receiver smart pointer used for can data reception

    std::shared_ptr<holo::sensors::candev::CanReceiver> receiver_ptr_;

    void Message_Encode(struct Config_SendCommand& input);
    bool BuildUdpConnection();
    //! @brief load parameters from yaml file
    //! @param[in] input full path string of input yaml file
    //! @return the result of loading parameters
    //! @retval true ok
    //! @retval false failed
    //! @note for the cpp-yaml, it crashed when target file or parameters
    //!       do not exist
    bool LoadParameters(const std::string& yaml_file);

    //! @brief parse front radar input can message and generate target object
    //!        info,and convert and transfer these data
    //! @param[in] msg the input can message
    //! @note this function usually installed to front_reciever_ptr_ through
    //!  ::holo::sensors::candev::CanReceiver::InstallMessageCallback
    //! @see ::holo::sensors::contiradar::Contiradar::CanMessageParsingFrontRadar
    void CanMessageParsingRadar(const holo::sensors::candev::CanMessage& msg, enum Radar_location radar_location);

};  // class Contiradar

}   // namespace contiradar
}   // namespace sensors
}   // namespace holo

#endif