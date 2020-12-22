/*!
* \brief This file defines pandar-64 class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#include <holo/sensors/pandar/pandar.h>
#include <glog/logging.h>

namespace holo
{
namespace sensors
{
namespace pandar
{

Pandar::Pandar(const std::string& config_yaml_file, const int32_t data_port, const int32_t gps_port, const std::string& coord)
{
    config_param_.data_port = data_port;
    config_param_.gps_port = gps_port;
    config_param_.coord = coord;

    LoadConfigFromYaml(config_yaml_file);
    base_hour_.sec = 0;
    base_hour_.nsec = 0;

    if(!BuildUdpConnection(config_param_.data_port, data_sockfd_))
    {
       return;
    }

    if(config_param_.use_gps)
    {
       if(!BuildUdpConnection(config_param_.gps_port, gps_sockfd_))
       {
          return;
       }
       gps_thread_ptr_ = std::shared_ptr<std::thread >(new std::thread(std::bind(
                           &Pandar::ReadSyncInfo, this)));
    }

}

void Pandar::Run()
{
    data_thread_ptr_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&Pandar::ReadDataInfo, this)));
}

void Pandar::ReadDataInfo()
{
    while(true)
    {
       if(!ReadDataMsgs())
       {
          return;
       }
    }
}

void Pandar::LoadConfigFromYaml(const std::string& yaml_file)
{
    yaml::Node root_node;
    root_node = yaml::LoadFile(yaml_file);
    config_param_.rpm = root_node["rpm"].as<float32_t>();
    config_param_.use_gps = root_node["use_gps"].as<bool>();
}

bool Pandar::BuildUdpConnection(const int32_t& port, int32_t& sockfd)
{
    sockfd = socket(PF_INET, SOCK_DGRAM, 0);

    if (sockfd == -1)
    {
        LOG(ERROR) << "could not creat socket for port: " << port;
        return false;
    }

    sockaddr_in data_addr;
    memset(&data_addr, 0, sizeof(data_addr));
    data_addr.sin_family = AF_INET;
    LOG(INFO) << "the port " << port;
    data_addr.sin_port = htons(port);
    data_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (sockaddr*)&data_addr, sizeof(sockaddr)) == -1)
    {
        LOG(ERROR) << "could not bind for port: " << port;
        return false;
    }

    if (fcntl(sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
        LOG(ERROR) << "non-block for port: " << port;
        return false;
    }
    return true;
}

bool Pandar::GetDataPacket(PandarPacket& data_packet)
{
    struct pollfd fds[1];
    fds[0].fd = data_sockfd_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true)
    {
        do
        {
            int retval = poll(fds, 1, POLL_TIMEOUT);

            if (retval < 0)
            {
                if (errno != EINTR)
                {
                    LOG(ERROR) << "poll() error: " << strerror(errno);
                }

                return false;
            }

            if (retval == 0)
            {
                LOG(WARNING) << "pandar poll() timeout";
            }

            if ((fds[0].revents & POLLERR) ||
                (fds[0].revents & POLLHUP) ||
                (fds[0].revents & POLLNVAL))
            {
                LOG(ERROR) << "poll() reports pandar error";
                return false;
            }
        }
        while ((fds[0].revents & POLLIN) == 0); // loop if there is no input data received

        ssize_t nbytes = recvfrom(data_sockfd_, data_packet.Data(),
                                  PANDAR_PACKET_BYTE_SIZE,  0,
                                  (sockaddr*) &sender_address, &sender_address_len);
        //LOG(INFO) << "BYTES:" << nbytes;
        if (nbytes < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                LOG(ERROR) << "recvfail";
                return false;
            }
        }
        else if ((size_t) nbytes == PANDAR_PACKET_BYTE_SIZE)
        {
            break;
        }

        LOG(INFO) << "incomplete pandar packet read: "
                  << nbytes << " bytes";
    }

    return true;
}

bool Pandar::ReadDataMsgs()
{
    PandarPacket data_packet;
    PandarScan scan;

    /* assign the coord */
    scan.coord = config_param_.coord;

    /* NOTE: stl::vector::clear() does not reduce capacity() */
    scan.packets.clear();

    float32_t start_azimuth = 0.0;
    float32_t previous_azimuth = 0.0;
    float32_t current_azimuth = 0.0;
    /*FIXME unused variable*/
    /*uint32_t start_pandar_timestamp = 0.0;*/
    /*uint32_t current_pandar_timestamp = 0.0;*/

    while ( true )
    {
        /* get a packet from the input source */
        while (true)
        {
            if(GetDataPacket(data_packet))
            {
               break;
            }
        }

        /* cache the first packet */
        scan.packets.push_back(data_packet);
        /* initialize the azimuth and timestamp */
        if (scan.packets.size() == 1)
        {
            start_azimuth = previous_azimuth = current_azimuth = data_packet.blocks[0].Azimuth();
            continue;
        }
        for (int i =0; i< 6; i++)
        {
            //LOG(INFO) << "AZIMUTH ANGLE:" << i << data_packet.blocks[i].Azimuth();
        }
        
        /* check the azimuth when the half of expected duration has passed,
         * this help avoid the wrap around problem happening in the beginning */
        current_azimuth = data_packet.blocks[PANDAR_BLOCKS_PER_PACKET-1].Azimuth();
        //LOG(INFO) << "CURRENT ANGLE:" << current_azimuth;
        //LOG(INFO) << "START ANGLE:" << start_azimuth;
        if (current_azimuth >= previous_azimuth )  /* regular case */
        {
            if ( current_azimuth >= start_azimuth && previous_azimuth < start_azimuth )
            {
                break;
            }
        }
        else /* wrap around case */
        {
            /* the first condition is used when "start_azimuth" is closer to 0, while
                * the second condition is used when "start_azimuth" is closer to 360 */
            if ( (current_azimuth >= start_azimuth && (previous_azimuth-360.0) < start_azimuth) ||
                    ((current_azimuth+360.0) >= start_azimuth && (previous_azimuth <= start_azimuth)) )
            {
                break;
            }
        }
        previous_azimuth = current_azimuth;
    }

    //generate a timestamp for a complete scan. the timestamp is the time out of hour.

    if (config_param_.use_gps)
    {
        if (!nmea_received_)
        {
            LOG(ERROR) << "could not get the nmea signal";
        }
        if (!pps_received_)
        {
            LOG(ERROR) << "could not get the pps signal";

        }
    }

    {
       std::unique_lock<std::mutex> lck(base_hour_mutex_);
       scan.timestamp.sec = base_hour_.sec;
       scan.timestamp.nsec = base_hour_.nsec;
    }

    if(pandar_scan_callback_)
    {
        pandar_scan_callback_(scan);
    }
    // timestamp could be from gps or the default value

    return true;
}

bool Pandar::GetGpsPacket(PandarGpsPacket& gps_packet)
{
    struct pollfd fds[1];
    fds[0].fd = gps_sockfd_;
    fds[0].events = POLLIN;
    static const int32_t POLL_TIMEOUT = 2000;

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true)
    {
        do
        {
            int retval = poll(fds, 1, POLL_TIMEOUT);

            if (retval < 0)
            {
                if (errno != EINTR)
                {
                    LOG(ERROR) << "poll() error for gps packet: " << strerror(errno);
                }

                return false;
            }

            if (retval == 0)
            {
                LOG(WARNING) << "gps packet poll() timeout";
            }

            if ((fds[0].revents & POLLERR) ||
                (fds[0].revents & POLLHUP) ||
                (fds[0].revents & POLLNVAL))
            {
                LOG(ERROR) << "poll() reports gps error";
                return false;
            }
        }
        while ((fds[0].revents & POLLIN) == 0);

        ssize_t nbytes = recvfrom(gps_sockfd_, &gps_packet.header,
                                  PANDAR_GPS_PACKET_SIZE,  0,
                                  (sockaddr*) &sender_address, &sender_address_len);

        if (nbytes < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                LOG(ERROR) << "recvfail";
                return false;
            }
        }
        else if ((size_t) nbytes == PANDAR_GPS_PACKET_SIZE)
        {
             break;
        }

        LOG(INFO) << "incomplete position packet read: "
                  << nbytes << " bytes";
    }

    return true;
}

bool Pandar::ReadSyncInfo()
{
    PandarGpsPacket gps_packet;

    /* FIXME unused variable */
    /* uint16_t validity_index = 0, hour_index = 0, date_index = 0; */

    while (true)
    {
        if (!GetGpsPacket(gps_packet))
        {
            LOG(ERROR) << "could not get the position packet";
            nmea_received_ = false;
            continue;
        }
        else
        {
            if(gps_packet.pps_locked != 1)
            {
                LOG(INFO) << "pps is not locked ";
                pps_received_ = false;
                continue;
            }

            // the received is not an "A"
            if(gps_packet.location_valid != 65)
            {
                LOG(INFO) << "no nmea received";
                nmea_received_ = false;
                continue;
            }

            pps_received_ = true;
            nmea_received_ = true;

            static tm time;
            time.tm_year = (gps_packet.date[1] - 48) * 10
                         + (gps_packet.date[0] - 48) + 2000 - 1900;
            time.tm_mon  = (gps_packet.date[3] - 48) * 10
                         +  gps_packet.date[2] - 48 - 1;
            time.tm_mday = (gps_packet.date[5] - 48) * 10
                         +  gps_packet.date[4] - 48;
            time.tm_hour = (gps_packet.time[5] - 48) * 10
                         +  gps_packet.time[4] - 48;
            time.tm_min  = (gps_packet.time[3] - 48) * 10
                         +  gps_packet.time[2] - 48;
            time.tm_sec  = (gps_packet.time[1] - 48) * 10
                         +  gps_packet.time[0] - 48;

            /// this function considers input as UTC/GMT, not local time zone
            /// base_hour_ will store the utc stampstamp converted from $GPRMC in seconds
            {
                std::unique_lock<std::mutex> lck(base_hour_mutex_);
                {
                    float64_t sec = static_cast<float64_t>(timegm(&time));
                    if(sec < 0 || sec >std::numeric_limits<uint32_t>::max())
                    {
                        LOG(ERROR) << "base hour is incorrect " << sec;
                        LOG(ERROR) << "gps time year " << time.tm_year
                                   << "         mont " << time.tm_mon
                                   << "         mday " << time.tm_mday
                                   << "         hour " << time.tm_hour
                                   << "         min "  << time.tm_min
                                   << "         sec "  << time.tm_sec;
                        LOG(ERROR) << "********************";
                    }
                    else
                    {
                            int64_t sec64 = (int64_t)std::floor(sec);
                            if(sec64 < 0 || sec64 >std::numeric_limits<uint32_t>::max())
                            {
                                LOG(ERROR) << "TimeStamp is out of range";
                            }
                            base_hour_.sec = (uint32_t)sec64;
                            base_hour_.nsec = (uint32_t)((sec - base_hour_.sec) * 1e9);
                            base_hour_.sec += base_hour_.nsec / 1000000000;
                            base_hour_.nsec %= 1000000000;
                    }
                }
            }
        }
    }
}

}
}
}
