/*!
* \brief This file defines velodyne-16 class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef _HOLO_SENSORS_VELODYNE_H
#define _HOLO_SENSORS_VELODYNE_H

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <mutex>
#include <thread>

#include <holo/sensors/velodyne/velodyne_structure.h>
#include <holo/yaml.h>


namespace holo
{
namespace sensors
{
namespace velodyne
{

class Velodyne
{
public:
     // callback if velodyne scan
     typedef std::function<void(VelodyneScan&)> VelodyneScanCallback;

     // will be call in the driver to get the velodyne scan
     void SetVelodyneScanCallback(VelodyneScanCallback handler)
     {
        velodyne_scan_callback_ = handler;
     }

     // the config parameters for velodyne driver
     struct ConfigParameter
     {
        float32_t rpm;                    // the rotation per minute which is related to frequency
        std::string model;                // the model of hardware: vlp16 vlp32a...
        bool use_gps;                     // if the hardware is sync with gps and all timestamp would be gps time
        int32_t data_port;                // the port of packet for hardware
        int32_t gps_port;                 // the port of gps info for hardware
        std::string coord;                // the coordinate of velodyne scan
        float32_t packet_rate;            // indicates how many packets would be in one scan
        uint16_t expected_packet_num;     // the expected packet num in one scan relate to the hardware
        float32_t minimum_duration;       // the minimum duration for two packets
     };
     
     Velodyne() = delete;
     
     // the construction function for velodyne driver
     // [input] config_yaml_file:  the yaml file to load parameters would be in holo_sensors/params
     // [input] data_port:   the data port of velodyne
     // [input] gps_port:    the gps port of velodyne
     // [input] coord:       the coord of velodyne
     Velodyne(const std::string& config_yaml_file, const int32_t data_port, const int32_t gps_port, const std::string& coord);
     
     // fire the driver
     void Run();

private:
     // load parameters from yaml file
     // [input] config_yaml_file:  the yaml file to load parameters would be in holo_sensors/params
     bool LoadConfigFromYaml(const std::string& config_yaml_file);
     
     // build udp connection with hardware
     // [input]　port:       the port of network
     // [output] sockfd:    the sock id of udp connection
     // [return]:           return true if the udp is connected or return false
     bool BuildUdpConnection(const int32_t& port, int& sockfd);
     
     // get data packet from corresponding socket
     bool GetDataPacket(VelodynePacket& data_packet);
     void ReadDataInfo();
     bool ReadDataMsgs();
     
     // get gps packet from corresponding socket
     bool GetGpsPacket(VelodyneGpsRaw& gps_raw_packet);
     bool ReadSyncInfo();

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
     // a new thread to get data
     std::shared_ptr<std::thread> data_thread_ptr_;
     // mutex for base_hour
     std::mutex base_hour_mutex_;
     
     VelodyneScanCallback velodyne_scan_callback_;
};

}
}
}

#endif
