/*!
* \brief This file defines candev class for CAN message receive from eth(tcp/udp) interface.
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef _HOLO_CANDEV_H
#define _HOLO_CANDEV_H

#include <unistd.h>
#include <stdio.h>
#include <arpa/inet.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <exception.hpp>
#include <inetserverstream.hpp>
#include <inetserverdgram.hpp>

#include <holo/types.h>
#include <holo/msg_header.h>
#include <holo/sensors/candev/can_source.h>

namespace holo
{
namespace sensors
{
namespace candev
{

//! @brief can package layout from gateway board
struct CanPackage
{
    uint8_t             header[4];      //!< const string '##@!'
    uint8_t             channel;        //!< the hardware chn
    uint8_t             source;         //!< the messages source
    uint8_t             dlc;            //!< data length code
    uint8_t             unused0;        //!< unused byte
    uint32_t            id;             //!< can message identifier
    uint8_t             data[8];        //!< can message data
    uint32_t            timestamp_h;    //!< timestamp sec
    uint32_t            timestamp_l;    //!< timestamp usec
};

//! @brief can message definition
//!
//! used for the holo platform, independent of any hardware
struct CanMessage
{
    uint32_t            sec;            //!< timestamp sec
    uint32_t            nsec;           //!< timestamp usec
    uint32_t            channel;        //!< channel used
    CanSource           source;         //!< message source #CanSource
    uint32_t            id;             //!< message id
    uint32_t            dlc;            //!< message dlc
    uint8_t             data[8];        //!< message data
};

//! @brief build a CAN message receiver base on provided socket params
//!
//! @note generally the class may be used like this
//! @code
//! CanReceiver("udp", "localhost", "50000", "192.168.1.101", "50010");
//! CanReceiver("TCP", "192.168.1.3", "50000", "192.168.1.101", "50010");
//! @endcode
class CanReceiver
{
    //! can message service callbacl function protype
    typedef std::function<void(const holo::sensors::candev::CanMessage&)>
    CanMessageCallback;

public:
    //! @brief constructor of can receiver class
    //! @param[in] socket_type "tcp"/"TCP"/"udp"/"UDP" for specific type
    //! @param[in] host_name host ip addr or domain name
    //! @param[in] host_port host port used for communication
    //! @param[in] client_name client ip addr
    //! @param[in] client_port client port used for communication
    CanReceiver(std::string socket_type, std::string host_name,
                std::string host_port, std::string client_name,
                std::string client_port);

    //! @brief install can message service callback
    //! @param[in] callback callback fucntion reference
    //! @note this 'callback' will be called when a can message is received,
    //!       it's the duty to parsing or process the contained signals, based
    //!       on maybe the channel/source/id/dlc.
    void InstallMessageCallback(const CanMessageCallback& callback);

private:
    static const uint32_t kUtcGpsGap = 315964800; ///< Time gap between UTC and GPS in seconds
    static const uint32_t kGpsLeapSecond = 18; ///< GPS leap second
    //! socket communication buffer size
    const uint32_t buffer_size_ = 1024;
    //! can package header for data stream synchronization
    const std::string can_message_header_ = "##@!";

    //! local string of socket type
    std::string socket_type_;
    //! local string of host name
    std::string host_name_;
    //! local string of host port
    std::string host_port_;
    //! local string of client name
    std::string client_name_;
    //! local string of client port
    std::string client_port_;

    //! smart pointer of comminication thread
    std::shared_ptr<std::thread> recv_thread_ptr_;

    //! can service callback function, initialized as null,
    //! will not be called before given a value correctlly
    CanMessageCallback can_message_callback_;

    //! @brief tcp receive communication thread
    void TcpRecvThread();
    //! @brief udp receive communication thread
    void UdpRecvThread();
    //! @brief decoding can package and construct a corresponding can message
    //! @param[in] message the data stream that contains several can packages
    void MsgDecode(const std::string& message) const;

};  // class TcpReceive

}   // namespace candev
}   // namespace sensors
}   // namespace holo

#endif
