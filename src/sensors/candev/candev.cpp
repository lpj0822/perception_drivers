/*!
* \brief defines candev class for CAN message receive from eth(tcp/udp) interface.
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */


#include <unistd.h>
#include <arpa/inet.h>
#include <thread>

#include <inetserverstream.hpp>
#include <inetserverdgram.hpp>
#include <select.hpp>
#include <socket.hpp>

#include <glog/logging.h>
#include <holo/sensors/candev/candev.h>

namespace holo
{
namespace sensors
{
namespace candev
{

CanReceiver::CanReceiver(std::string socket_type, std::string host_name,
                         std::string host_port, std::string client_name,
                         std::string client_port):
    socket_type_(socket_type),
    host_name_(host_name),
    host_port_(host_port),
    client_name_(client_name),
    client_port_(client_port),
    can_message_callback_(0)
{
    if ((socket_type == std::string("tcp")) || (socket_type == std::string("TCP")))
    {
        recv_thread_ptr_ = std::shared_ptr<std::thread>(new std::thread(std::bind(
                               &CanReceiver::TcpRecvThread, this)));
    }
    else if ((socket_type == std::string("udp"))
             || (socket_type == std::string("UDP")))
    {
        recv_thread_ptr_ = std::shared_ptr<std::thread>(new std::thread(std::bind(
                               &CanReceiver::UdpRecvThread, this)));
    }
    else
    {
        LOG(ERROR) << "[CanReceiver] Invalid socket_type " << socket_type << " found!";
    }
}

void CanReceiver::TcpRecvThread()
{
    try
    {
        libsocket::inet_stream_server srv(host_name_, host_port_, LIBSOCKET_IPv4);

        libsocket::selectset<libsocket::inet_stream_server> sset;
        sset.add_fd(srv, LIBSOCKET_READ);

        /********* SELECT PART **********/
        libsocket::selectset<libsocket::inet_stream_server>::ready_socks readypair;
        readypair = sset.wait(); // Wait for a connection and save the pair to the var

        libsocket::inet_stream_server* ready_srv;
        ready_srv = dynamic_cast<libsocket::inet_stream_server*>
                    (readypair.first.back());

        readypair.first.pop_back(); // delete the fd from the pair
        /*******************************/

        std::unique_ptr<libsocket::inet_stream> client;
        std::string message;

        while (1)
        {
            /*!
                will block here to accept socket connection(s)
            */
            client = ready_srv->accept2();

            while (1)
            {
                /*!
                    set receive buffer size and continue blocking-receive
                */
                message.resize(buffer_size_);
                *client >> message;

                if (message.length() <= 0)
                {
                    /*!
                        when the socket is corrupted, break to accept another one
                    */
                    break;
                }

                /* post-receive service */
                MsgDecode(message);
            }
        }

        srv.destroy();

    }
    catch (const libsocket::socket_exception& exc)
    {
        LOG(INFO) << "[CanReceiver]" << exc.mesg << std::endl;
    }
}

void CanReceiver::UdpRecvThread()
{
    std::string message;

    try
    {
        libsocket::inet_dgram_server srv(host_name_, host_port_, LIBSOCKET_IPv4);

        while (1)
        {
            /*!
                set receive buffersize and continue blocking-receive
            */
            message.resize(buffer_size_);
            srv.rcvfrom(message, client_name_, client_port_, 0, true);

            /* post-receive service */
            MsgDecode(message);
        }
    }
    catch (const libsocket::socket_exception& exc)
    {
        LOG(INFO) << "[CanReceiver]" << exc.mesg << std::endl;
    }
}

void CanReceiver::MsgDecode(const std::string& message) const
{
    char* pos = const_cast<char*>(message.c_str());
    CanPackage* pack = 0;
    CanMessage output;

    while (pos)
    {
        pos = strstr(pos, can_message_header_.c_str());
        if (pos)
        {
            pack = reinterpret_cast<CanPackage*>(pos);

#if 0 /* ONLY AT DEBUG */
            {
                LOG(INFO) << std::dec
                          << uint32_t(pack->channel) << "#"
                          << std::hex << std::noshowbase
                          << ntohl(pack->id) << "#"
                          << std::dec
                          << uint32_t(pack->dlc) << "="
                          << std::noshowbase
                          << std::hex << std::setw(2) << std::setfill('0')
                          << uint32_t(pack->data[0]) << " "
                          << std::hex << std::setw(2) << std::setfill('0')
                          << uint32_t(pack->data[1]) << " "
                          << std::hex << std::setw(2) << std::setfill('0')
                          << uint32_t(pack->data[2]) << " "
                          << std::hex << std::setw(2) << std::setfill('0')
                          << uint32_t(pack->data[3]) << " "
                          << std::hex << std::setw(2) << std::setfill('0')
                          << uint32_t(pack->data[4]) << " "
                          << std::hex << std::setw(2) << std::setfill('0')
                          << uint32_t(pack->data[5]) << " "
                          << std::hex << std::setw(2) << std::setfill('0')
                          << uint32_t(pack->data[6]) << " "
                          << std::hex << std::setw(2) << std::setfill('0')
                          << uint32_t(pack->data[7]);
            }
#endif

            /* callback, for maybe ros-message broadcast */
            if (can_message_callback_)
            {
                /* convert utc time to gps time */
                Timestamp utc_stamp;
                int64_t sec = ntohl(pack->timestamp_h);
                int64_t nsec = ntohl(pack->timestamp_l);
                utc_stamp = Normalize(sec, nsec);
                Timestamp gps_stamp;
                gps_stamp = UtcToGps(utc_stamp);

                output.sec     = gps_stamp.sec;
                output.nsec    = gps_stamp.nsec;
                output.channel = pack->channel;
                output.source  = CanSource(pack->source);
                output.id      = ntohl(pack->id);
                output.dlc     = pack->dlc;
                output.data[0] = pack->data[0];
                output.data[1] = pack->data[1];
                output.data[2] = pack->data[2];
                output.data[3] = pack->data[3];
                output.data[4] = pack->data[4];
                output.data[5] = pack->data[5];
                output.data[6] = pack->data[6];
                output.data[7] = pack->data[7];

                can_message_callback_(output);
            }

            pos += sizeof(struct CanPackage);
        }
    }
}

void CanReceiver::InstallMessageCallback(const CanMessageCallback& callback)
{
    can_message_callback_ = callback;
}

}   // namespace candev
}   // namespace sensors
}   // namespace holo
