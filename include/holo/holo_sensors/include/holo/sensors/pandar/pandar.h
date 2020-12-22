/*!
* \brief This file defines pandar-64 class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef _HOLO_SENSORS_PANDAR_H
#define _HOLO_SENSORS_PANDAR_H

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <mutex>
#include <thread>
#include <holo/yaml.h>
#include <holo/sensors/pandar/pandar_structure.h>

namespace holo
{
namespace sensors
{
namespace pandar
{

class Pandar
{
public:
     // callback if pandar scan
     typedef std::function<void(PandarScan&)> PandarScanCallback;

     // will be call in the driver to get the pandar scan
     void SetPandarScanCallback(PandarScanCallback handler)
     {
        pandar_scan_callback_ = handler;
     }

     // the config parameters for pandar driver
     struct ConfigParameter
     {
        float32_t rpm;                    // the rotation per minute which is related to frequency
        bool use_gps;                     // if the hardware is sync with gps and all timestamp would be gps time
        int32_t data_port;                // the port of packet for hardware
        int32_t gps_port;                 // the port of gps info for hardware
        std::string coord;                // the coordinate of pandar scan
     };

     Pandar() = delete;

     // the construction function for pandar driver
     // [input] config_yaml_file:  the yaml file to load parameters would be in holo_sensors/params
     // [input] data_port:   the data port of pandar
     // [input] gps_port:    the gps port of pandar
     // [input] coord:       the coord of pandar
     Pandar(const std::string& config_yaml_file, const int32_t data_port, const int32_t gps_port, const std::string& coord);

     // fire the driver
     void Run();

private:
     // load parameters from yaml file
     // [input] config_yaml_file:  the yaml file to load parameters would be in holo_sensors/params
     void LoadConfigFromYaml(const std::string& config_yaml_file);

     // build udp connection with hardware
     // [input]　port:       the port of network
     // [output] sockfd:    the sock id of udp connection
     // [return]:           return true if the udp is connected or return false
     bool BuildUdpConnection(const int32_t& port, int32_t& sockfd);

     // get data packet from corresponding socket
     bool GetDataPacket(PandarPacket& data_packet);
     bool ReadDataMsgs();

     // get gps packet from corresponding socket
     bool GetGpsPacket(PandarGpsPacket& gps_packet);
     bool ReadSyncInfo();
     void ReadDataInfo();

     // the config parameters
     ConfigParameter config_param_;
     // the sock id for data and gps
     int data_sockfd_, gps_sockfd_;
     // indicates if nmea and pps signals are received from gps port, which is needed for sync
     bool nmea_received_, pps_received_;
     // the hour timestamp for a whole scam
     Timestamp base_hour_;
     // a new thread to get gps info
     std::shared_ptr<std::thread> gps_thread_ptr_;

     // a new thread to get data info
     std::shared_ptr<std::thread> data_thread_ptr_;
     // mutex for base_hour
     std::mutex base_hour_mutex_;

     PandarScanCallback pandar_scan_callback_;
};

}
}
}

#endif
