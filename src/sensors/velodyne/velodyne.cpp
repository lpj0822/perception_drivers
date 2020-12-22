/*!
* \brief This file defines velodyne class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#include <glog/logging.h>
#include <holo/sensors/velodyne/velodyne.h>
#include <holo/sensors/velodyne/velodyne_structure.h>

namespace holo
{
namespace sensors
{
namespace velodyne
{

Velodyne::Velodyne(const std::string& config_yaml_file, const int32_t data_port, const int32_t gps_port, const std::string& coord)
{
    config_param_.data_port = data_port;
    config_param_.gps_port = gps_port;
    config_param_.coord = coord;

    LoadConfigFromYaml(config_yaml_file);
    base_hour_.sec = 0;
    base_hour_.nsec = 0;

    if (config_param_.model == "VLP16")
    {
        config_param_.packet_rate = 760.0;
        
    }
    else
    {
        LOG(ERROR) << "unsupported Velodyne model" << config_param_.model;
        return;
    }

    float32_t sec_per_rotation = 1/ (config_param_.rpm/60);
    config_param_.expected_packet_num = config_param_.packet_rate * sec_per_rotation;
    config_param_.minimum_duration = 0.75 * sec_per_rotation;
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
                           &Velodyne::ReadSyncInfo, this)));
    }

}

void Velodyne::Run()
{
    data_thread_ptr_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&Velodyne::ReadDataInfo, this)));
    
}

void Velodyne::ReadDataInfo()
{
    while(true)
    {
       if(!ReadDataMsgs())
       {
          return;
       }
    }
}

bool Velodyne::LoadConfigFromYaml(const std::string& config_yaml_file)
{
    const std::string string_rpm = "rpm";
    const std::string string_model = "model";
    const std::string string_use_gps = "use_gps";

    try
    {
        yaml::Node yaml_file            = yaml::LoadFile(config_yaml_file);
        config_param_.rpm               = yaml_file[string_rpm].as<float32_t>();
        config_param_.model             = yaml_file[string_model].as<std::string>();
        config_param_.use_gps           = yaml_file[string_use_gps].as<bool>();
    }
    catch(std::exception& e)
    {
        return false;
    }

    return true;
}    

bool Velodyne::BuildUdpConnection(const int32_t& port, int& sockfd)
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

bool Velodyne::GetDataPacket(VelodynePacket& data_packet)
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
                LOG(WARNING) << "Velodyne poll() timeout";
            }

            if ((fds[0].revents & POLLERR) ||
                (fds[0].revents & POLLHUP) ||
                (fds[0].revents & POLLNVAL))
            {
                LOG(ERROR) << "poll() reports Velodyne error";
                return false;
            }
        }
        while ((fds[0].revents & POLLIN) == 0); // loop if there is no input data received

        ssize_t nbytes = recvfrom(data_sockfd_, data_packet.Data(),
                                  VELODYNE_PACKET_BYTE_SIZE,  0,
                                  (sockaddr*) &sender_address, &sender_address_len);

        if (nbytes < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                LOG(ERROR) << "recvfail";
                return false;
            }
        }
        else if ((size_t) nbytes == VELODYNE_PACKET_BYTE_SIZE)
        {
            
            break;
        }

        LOG(INFO) << "incomplete Velodyne packet read: "
                  << nbytes << " bytes";
    }

    return true;
}

bool Velodyne::ReadDataMsgs()
{
    VelodynePacket data_packet;
    VelodyneScan scan;

    /* assign the coord */
    scan.coord_id = config_param_.coord;

    /* NOTE: stl::vector::clear() does not reduce capacity() */
    scan.packets.clear();
    if ( scan.packets.capacity() < config_param_.expected_packet_num)
    {
        scan.packets.reserve(config_param_.expected_packet_num);
    }

    float32_t start_azimuth = 0.0;
    float32_t previous_azimuth = 0.0;
    float32_t current_azimuth = 0.0;
    uint32_t start_velodyne_timestamp = 0.0;
    uint32_t current_velodyne_timestamp = 0.0;

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

        /* check the number of packets, if already more than expected, break. */
        if ( scan.packets.size() >= config_param_.expected_packet_num)
        {
            break;
        }

        /* initialize the azimuth and timestamp */
        if ( scan.packets.size() == 1 )
        {
            start_azimuth = previous_azimuth = current_azimuth = data_packet.blocks[0].Azimuth();
            start_velodyne_timestamp = current_velodyne_timestamp = data_packet.velodyne_timestamp;
            continue;
        }

        /* check whether the time difference to the first packet has passed starting timestamp */
        current_velodyne_timestamp = data_packet.velodyne_timestamp;
        float64_t t =
           fmod((float64_t) current_velodyne_timestamp + 3600.0*1000000.0 - (float64_t) start_velodyne_timestamp, 3600.0*1000000.0) / 1000000.0;

        /* check the azimuth when the half of expected duration has passed,
         * this help avoid the wrap around problem happening in the beginning */
        current_azimuth = data_packet.blocks[VELODYNE_BLOCKS_PER_PACKET-1].Azimuth();

        if ( t >= config_param_.minimum_duration )
        {
            if ( current_azimuth >= previous_azimuth )  /* regular case */
            {
                if ( current_azimuth >= start_azimuth && previous_azimuth <= start_azimuth )
                {
                    break;
                }
            }
            else /* wrap around case */
            {
                /* the first condition is used when "start_azimuth" is closer to 0, while
                 * the second condition is used when "start_azimuth" is closer to 360 */
                if ( (current_azimuth >= start_azimuth && (previous_azimuth-360.0) <= start_azimuth) ||
                     ((current_azimuth+360.0) >= start_azimuth && (previous_azimuth <= start_azimuth)) )
                {
                    break;
                }
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
       scan.timestamp = base_hour_;
    }

    if(velodyne_scan_callback_)
    {
       velodyne_scan_callback_(scan);
    }

    return true;
}

bool Velodyne::GetGpsPacket(VelodyneGpsRaw& gps_raw_packet)
{
    struct pollfd fds[1];
    fds[0].fd = gps_sockfd_;
    fds[0].events = POLLIN;
    static const int32_t POLL_TIMEOUT = 1000;

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

        ssize_t nbytes = recvfrom(gps_sockfd_, &gps_raw_packet.unused_1[0],
                                  VELODYNE_GPS_PACKET_BYTE_SIZE,  0,
                                  (sockaddr*) &sender_address, &sender_address_len);

        if (nbytes < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                LOG(ERROR) << "recvfail";
                return false;
            }
        }
        else if ((size_t) nbytes == VELODYNE_GPS_PACKET_BYTE_SIZE)
        {
             break;
        }

        LOG(INFO) << "incomplete position packet read: "
                  << nbytes << " bytes";
    }

    return true;
}

bool Velodyne::ReadSyncInfo()
{
    VelodyneGpsRaw gps_raw_packet;
    uint16_t validity_index = 0, hour_index = 0, date_index = 0;

    while (true)
    {
        if (!GetGpsPacket(gps_raw_packet))
        {
            LOG(ERROR) << "could not get the position packet";
            nmea_received_ = false;
            return false;
        }
        else
        {
            VelodyneSync gps_sync;
            gps_sync.pps_state = gps_raw_packet.pps_state;
            
            gps_sync.gps_timestamp = (gps_raw_packet.timestamp[3] << 24) |
                                     (gps_raw_packet.timestamp[2] << 16) | 
                                     (gps_raw_packet.timestamp[1] << 8) |
                                     (gps_raw_packet.timestamp[0]);
            
            if (gps_sync.pps_state !=2)
            {
              LOG(INFO) << "pps signal is not received";
              pps_received_ = false;
            }
            else
            {
              pps_received_ = true;
            }
            
            //the information we got from the postion port is a string includes different contents "hhmmss"s
            //different contents are seperated by ","
            //so parsing the gps information is to find the postion of "," in this string
            //and the index of of different content could be found based on these positions
            // this is an example: GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
            int index = 0;

            for (int i = 0; i != VELODYNE_NMEA_BYTE_SIZE; i++)
            {
                //44 is the "," in ASCII. Finding it would get the corresponding index
                if (gps_raw_packet.nmea[i] == 44)
                {
                    index = index + 1;

                    if (index == 1)
                    {
                        // the hhmmss is after the first ","
                        hour_index = i + 1;
                    }
                    else if (index == 2)
                    {
                        // if velodyne receives gps signal, this byte would be "A" if a valid signal is received
                        validity_index = i + 1;
                    }
                    else if (index == 9)
                    {
                        date_index = i + 1;
                    }
                }
            }
            //the validity is a "A" which means the hardware receives gps signal
            if (gps_raw_packet.nmea[validity_index] == 65)
            {
                gps_sync.nmea_state = 1;
                nmea_received_ = true;
                static tm time;
                time.tm_year = (gps_raw_packet.nmea[date_index + 4] - 48) * 10
                             + (gps_raw_packet.nmea[date_index + 5] - 48) + 2000 - 1900;
                time.tm_mon = (gps_raw_packet.nmea[date_index + 2] - 48) * 10
                            +  gps_raw_packet.nmea[date_index + 3] - 48 - 1;
                time.tm_mday = (gps_raw_packet.nmea[date_index] - 48) * 10
                             + gps_raw_packet.nmea[date_index + 1] - 48;
                time.tm_hour = (gps_raw_packet.nmea[hour_index] - 48) * 10
                             + gps_raw_packet.nmea[hour_index + 1] - 48;
                time.tm_min = 0;
                time.tm_sec = 0;

                /// this function considers input as UTC/GMT, not local time zone
                /// base_hour_ will store the utc stampstamp converted from $GPRMC in seconds
                {
                    std::unique_lock<std::mutex> lck(base_hour_mutex_);

                    float64_t tmp_time = static_cast<float64_t>(timegm(&time));

                    int64_t sec64 = (int64_t)std::floor(tmp_time);
                    if(sec64 < 0 || sec64 >std::numeric_limits<uint32_t>::max())
                    {
                        LOG(ERROR) << "TimeStamp is out of 32-bit range";
                    }
                    uint32_t sec = (uint32_t)sec64;
                    uint32_t nsec = (uint32_t)((tmp_time - sec) * 1e9);
                    sec += nsec / 1000000000;
                    nsec %= 1000000000;

                    base_hour_.sec = sec;
                    base_hour_.nsec = nsec;
                }
            }
            else if (gps_raw_packet.nmea[validity_index] == 86)
            {
                gps_sync.nmea_state = 0;
                nmea_received_ = false;
            }
            else
            {
                gps_sync.nmea_state = 2;
                nmea_received_ = false;
            }
            
        }
    }
}

}
}
}