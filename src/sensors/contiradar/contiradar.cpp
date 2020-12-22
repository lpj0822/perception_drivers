/*!
* \brief This file defines Contiradar class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#include <holo/sensors/contiradar/contiradar.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>

#include <glog/logging.h>
#include <holo/sensors/contiradar/contiradar_dbc.h>

namespace holo
{
namespace sensors
{
namespace contiradar
{

Contiradar::Contiradar(const std::string& yaml_file)
{
    memset(number_of_objects_, 0, sizeof(number_of_objects_));
    memset(index_part1_, 0, sizeof(index_part1_));
    memset(index_part2_, 0, sizeof(index_part2_));
    memset(index_part3_, 0, sizeof(index_part3_));
    memset(first_time_object_, 0, sizeof(first_time_object_));
    memset(first_time_object_part1_, 0, sizeof(first_time_object_part1_));
    memset(ars048_obstacles_callback_, 0, sizeof(ars048_obstacles_callback_));
    
    /* load working parameters */ 
    if(!LoadParameters(yaml_file))
    {
        LOG(FATAL) << "Failed to load contiradar config parameters!";
    }
    else
    {
        BuildUdpConnection();
    } 

    receiver_ptr_ = std::make_shared<holo::sensors::candev::CanReceiver>(
        socket_type_, host_ip_, host_port_, client_ip_, client_port_);

    /* create can receiver for front radar */
    if (radar_location_ == "front center")
    {
        receiver_ptr_->InstallMessageCallback(
            std::bind(&Contiradar::CanMessageParsingRadar, this, std::placeholders::_1, Radar_location::FRONT)
            );
    } 
    // else if (radar_location_ == "corner left")
    // {
    //     receiver_ptr_->InstallMessageCallback(
    //         std::bind(&Contiradar::CanMessageParsingRadar, this, std::placeholders::_1, Radar_location::CORNER_LEFT)
    //         );
    // }
    // else if (radar_location_ == "corner right")
    // {
    //     receiver_ptr_->InstallMessageCallback(
    //         std::bind(&Contiradar::CanMessageParsingRadar, this, std::placeholders::_1, Radar_location::CORNER_RIGHT)
    //         );
    // }
    // else if (radar_location_ == "rear left")
    // {
    //     receiver_ptr_->InstallMessageCallback(
    //         std::bind(&Contiradar::CanMessageParsingRadar, this, std::placeholders::_1, Radar_location::REAR_LEFT)
    //         );
    // }
    // else if (radar_location_ == "rear right")
    // {
    //     receiver_ptr_->InstallMessageCallback(
    //         std::bind(&Contiradar::CanMessageParsingRadar, this, std::placeholders::_1, Radar_location::REAR_RIGHT)
    //         );
    // }
    else if (radar_location_ == "rear center")
    {
        receiver_ptr_->InstallMessageCallback(
            std::bind(&Contiradar::CanMessageParsingRadar, this, std::placeholders::_1, Radar_location::REAR)
            );
    }
    else
    {
        LOG(FATAL) << "Failed to load contiradar location!";
    }                      
}

bool Contiradar::LoadParameters(const std::string& yaml_file)
{ 
    const std::string string_radar_location = "RADAR_LOCATION";
    const std::string string_socket_type = "SOCKET_TYPE";
    const std::string string_host_ip = "HOST_IP";
    const std::string string_host_port = "HOST_PORT";
    const std::string string_client_ip = "CLIENT_IP";
    const std::string string_client_port = "CLIENT_PORT";
    const std::string string_output_type = "OUTPUT_TYPE";
    const std::string string_send_quality = "SEND_QUALITY";
    const std::string string_send_extend = "SEND_EXTEND";
    const std::string string_max_distance = "MAX_DISTANCE";
    const std::string string_radar_power = "RADAR_POWER";
    const std::string string_rcs_threshold = "RCS_THRESHOLD";
    const std::string string_filter_index = "FILTER_INDEX";
    const std::string string_filter_type = "FILTER_TYPE";
    const std::string string_filter_min_value = "FILTER_MIN_VALUE";
    const std::string string_filter_max_value = "FILTER_MAX_VALUE";

    std::string yaml_path;
    yaml_path = yaml_file;
    yaml::Node root_node = yaml::LoadFile(yaml_path);

    radar_location_ = root_node[string_radar_location].as<std::string>();
    socket_type_    = root_node[string_socket_type].as<std::string>();
    host_ip_        = root_node[string_host_ip].as<std::string>();
    host_port_      = root_node[string_host_port].as<std::string>();
    client_ip_      = root_node[string_client_ip].as<std::string>();
    client_port_    = root_node[string_client_port].as<std::string>();
    output_type_    = root_node[string_output_type].as<std::string>();
    send_quality_   = root_node[string_send_quality].as<uint32_t>();
    send_extend_    = root_node[string_send_extend].as<uint32_t>();
    max_distance_   = root_node[string_max_distance].as<uint32_t>();
    radar_power_    = root_node[string_radar_power].as<uint32_t>();
    rcs_threshold_  = root_node[string_rcs_threshold].as<uint32_t>();
    filter_index_   = root_node[string_filter_index].as<uint32_t>();
    filter_type_    = root_node[string_filter_type].as<std::string>();
    filter_min_value_ = root_node[string_filter_min_value].as<float32_t>();
    filter_max_value_ = root_node[string_filter_max_value].as<float32_t>();

    return true;
}

void Contiradar::Message_Encode(struct Config_SendCommand& input)
{
    // header
    input.header[0] = '#';
    input.header[1] = '#';
    input.header[2] = '@';
    input.header[3] = '!';

    //radar_location
    if (radar_location_ == "front center")
    {
        input.radar_location = CenterFront_SRR;
    }
    // else if (radar_location_ == "corner left")
    // {
    //     input.radar_location = CornerLeft_SRR;
    // }
    // else if (radar_location_ == "corner right")
    // {
    //     input.radar_location = CornerRight_SRR;
    // }
    // else if (radar_location_ == "rear left")
    // {
    //     input.radar_location = RearLeft_SRR;
    // }
    // else if (radar_location_ == "rear right")
    // {
    //     input.radar_location = RearRight_SRR;
    // }
    else if (radar_location_ == "rear center")
    {
        input.radar_location = CenterRear_SRR;
    }
    else
    {
        //radar location unknown
        input.radar_location = 0;
        LOG(INFO) << "Radar location config error!";
    }

    //output type
    if (output_type_ == "OBJECT")
    {
        input.output_type = 1;
    }
    else if (output_type_ == "CLUSTER")
    {
        input.output_type = 2;
    }
    else if(output_type_ == "KEEP")
    {
        input.output_type = 3;
    } 
    else
    {
        //output type unknown
        input.output_type = 0;
        LOG(INFO) << "Output type config error!";
    }

    //send quality
    if (send_quality_ == 1)
    {
        input.send_quality = 1;
    }
    else
    {
        input.send_quality = 0;
    }

    //send extend
    if (send_extend_ == 1)
    {
        input.send_extend = 1;
    }
    else
    {
        input.send_extend = 0;
    }

    //max distance
    input.max_distance = max_distance_;

    //radar power
    if (radar_power_ == 1)
    {
        input.radar_power = 1;
    }
    else if (radar_power_ == 2)
    {
        input.radar_power = 2;
    }
    else if (radar_power_ == 3)
    {
        input.radar_power = 3;
    }
    else
    {
        input.radar_power = 0;
    }

    //rcs threshold
    if (rcs_threshold_ == 1)
    {
        input.rcs_threshold = 1;
    }
    else
    {
        input.rcs_threshold = 0;
    }

    //filter index
    input.filter_index = filter_index_;
    if (filter_index_ > 15)
    {
        LOG(INFO) << "filter index config error! The index is too big";
    }

    //filter type
    if (filter_type_ == "CLUSTER")
    {
        input.filter_type = 0;
    }
    else if (filter_type_ == "OBJECT")
    {
        input.filter_type = 1;
    }
    else if (filter_type_ == "KEEP")
    {
        input.filter_type = 2;
    }
    else
    {
        LOG(INFO) << "filter type config error!";
    }

    //filter min value
    input.filter_min_value = filter_min_value_;

    //filter max value
    input.filter_max_value = filter_max_value_;    
}

bool Contiradar::BuildUdpConnection()
{
    if ((sock_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        LOG(ERROR) << "Could not build socket";
        return false;
    }
    bzero(&addr_, sizeof(addr_));
    addr_.sin_family = AF_INET;
    uint32_t i = atoi(client_port_.c_str());
    addr_.sin_port = htons(i);
    const char* sock_ip = client_ip_.c_str();
    addr_.sin_addr.s_addr = inet_addr(sock_ip);
    if (addr_.sin_addr.s_addr == INADDR_NONE)
    {
        LOG(ERROR) << "Incorrect ip address!";
        close(sock_);
        return false;
    }

    Message_Encode(send_command);

    int n_bytes = sendto(sock_, &send_command.header[0], sizeof(send_command), 0, (struct sockaddr *)&addr_, sizeof(addr_));
    //LOG(INFO) << "Bytes:  " << n_bytes; 
    if (n_bytes < 0)
    {
        LOG(ERROR) << "could not send data to gw";
        return false;
    }
    close(sock_);
    LOG(INFO) << "Config success and close the socket";

    return true;
}

void Contiradar::CanMessageParsingRadar(const holo::sensors::candev::CanMessage& msg, enum Radar_location radar_location)
{
    switch(msg.id)
    {   
        if (output_type_ == "OBJECT")
        {
            CONTIRADAR_STORE_OBJECT(radar_location);
        }
        else if (output_type_ == "CLUSTER")
        {
            CONTIRADAR_STORE_CLUSTER(radar_location);
        }
    }
}

void Contiradar::InstallServiceRawObsCallback(const Ars048ObstaclesCallback& callback, int radar_index)
{
    if (radar_index == 0)
    {
        ars048_obstacles_callback_[0] = callback;
    }
    else if (radar_index == 1)
    {
        ars048_obstacles_callback_[1] = callback;
    }
    else if (radar_index == 2)
    {
        ars048_obstacles_callback_[2] = callback;
    }
    else if (radar_index == 3)
    {
        ars048_obstacles_callback_[3] = callback;
    }
    else if (radar_index == 4)
    {
        ars048_obstacles_callback_[4] = callback;
    }
    else if (radar_index == 5)
    {
        ars048_obstacles_callback_[5] = callback;
    }    
}

}   // namespace contiradar
}   // namespace sensors
}   // namespace holo

