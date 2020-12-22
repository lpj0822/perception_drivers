/*!
* \brief This file defines novatel class for message parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef _HOLO_NOVATEL_H
#define _HOLO_NOVATEL_H

#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <serial/serial.h>
#include <cmath>

#include <holo/msg_header.h>
#include <holo/sensors/novatel/novatel_structure.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

namespace holo
{
namespace sensors
{
namespace novatel
{
// this class is used as the driver for novatel and receives messages from hardware
class Novatel
{

public:
    // raw inspvax message callback
    typedef std::function<void(InsPositionVelocityAttitudeExtended&)> InsPvaXCallback;

    void SetInsPvaXCallback(InsPvaXCallback handler)
    {
        ins_pva_x_callback_ = handler;
    };

    // the construction function of the class
    // [input] dev:       the serial device name
    // [input] baud_rate: the serial baud rate
    //Novatel(const std::string& dev, const uint32_t& baud_rate);
    Novatel(const std::string& com_type, const std::string& ip, const uint32_t& port,
            const std::string& dev, const uint32_t& baud_rate, const std::string& cmd);

    // the destruction function
    ~Novatel();

    // get gps time from header
    // [input] header: the novatel header from messages
    // [return]:       timestamp from gps time
    Timestamp ConvertGpsToTimestamp(const Oem7Header& header);

private:
    // messages from hardware would be reviced in this function
    void SerialReadMessages();
    // the received data would be processed here and tell what kind of message is obtained
    // [input] msg:  the data we get from novatel
    // [input] len:  the length of message we received via tcp
    void ParseMessages(const unsigned char* msg, const unsigned int& len);

    // this function is used to connect the hardware via tcp
    // return true if the connection is successfully built, otherwise return false
    bool ConnectDevice();

    // sending commands to hardware
    // return true if commands are successfully sent, otherwise return false
    bool SetCommand();

    // messages from hardware would be reviced by tcp in this function
    void TcpReadMessages();

    // callbacks
    InsPvaXCallback ins_pva_x_callback_;
    // recorded file name
    std::string file_name_;
    // ip address
    std::string ip_, com_type_;
    // configuration command
    std::string cmd_;
    // recorded file name
    uint32_t port_;
    // sock structure

    struct sockaddr_in servaddr_;
    // connection socket
    int32_t sockfd_;

    std::shared_ptr<std::thread> read_thread_ptr_;

    std::shared_ptr<serial::Serial> serial_dev_;

    bool is_solution_good_;


    // define some const values to tell the command content for tcp sock
    float32_t command_sample_time_[1];

    static const uint8_t INSPVA_COMMAND_IDX      =   1;

    const std::string INSPVA_COMMAND_STR      = "inspvaxb";

    //serial data 
    int32_t msg_length_;
    unsigned char b_msg_[SERIAL_BUFFER_SIZE];
    unsigned char s_msg_[SERIAL_BUFFER_SIZE];
};

}
}
}

#endif
