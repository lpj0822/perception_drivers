/*!
* \brief This file defines image sync class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#include <holo/sensors/camera/image_sync.h>
#include <glog/logging.h>

namespace holo
{
namespace sensors
{
namespace camera
{

ImageSync::ImageSync(const std::string& yaml_file)
{

    if(!LoadParameters(yaml_file))
    {
    	return;
    }

    if(!SetUdpConnection())
    {
    	return;
    }

    image_sync_ipc_server_ = std::make_shared<ImageSyncIpc>("CAMERA_FRONT_CENTER", 0, 0, true, true);

    udp_thread_ptr_ = std::shared_ptr<std::thread >(new std::thread(std::bind(
                           &ImageSync::GetSyncInfo, this)));
}

bool ImageSync::LoadParameters(const std::string& yaml_file)
{
	try
	{
		yaml::Node root_node = yaml::LoadFile(yaml_file);
		ip_ = root_node["ip"].as<std::string>();
		port_ = root_node["port"].as<uint32_t>();
	}
	catch(std::exception& e)
	{
		LOG(ERROR) << "Could not load parameters for ImageSync";
		return false;
	}

	return true;
}

bool ImageSync::SetUdpConnection()
{
	if ((sock_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        LOG(ERROR) << "Could not build socket";
        return false;
    }

    bzero(&recv_addr_, sizeof(recv_addr_));
    recv_addr_.sin_family = AF_INET;
    recv_addr_.sin_port = htons(port_);
    recv_addr_.sin_addr.s_addr =  htonl(INADDR_ANY);

    if (bind(sock_, (struct sockaddr*)&recv_addr_, sizeof(recv_addr_)) == -1)
    {
        LOG(ERROR) << "Could not bind";
        LOG(INFO) << strerror(errno);
        return false;
    }
    return true;
}

void ImageSync::GetSyncInfo()
{
	unsigned int slen = sizeof(recv_addr_);
    uint32_t time_sec, time_nsec, pps_counter;
    while (true)
    {
        uint8_t data_recv[12];
        int16_t n_bytes = recvfrom(sock_, data_recv, 12, 0,
                                   (struct sockaddr*) &recv_addr_, &slen);
        if (n_bytes < 0)
        {
            LOG(ERROR) << "could not receive the data";
            close(sock_);
            break;
        }

        if (n_bytes == 0)
        {
            LOG(ERROR) << "server closed";
            close(sock_);
            break;
        }

        if (n_bytes > 0)
        {
            memcpy(&time_sec, &data_recv[0], 4);
            memcpy(&time_nsec, &data_recv[4], 4);
            memcpy(&pps_counter, &data_recv[8], 4);
         
            pps_counter = htonl(pps_counter);
            time_sec = htonl(time_sec); 
            time_nsec = htonl(time_nsec);
            // LOG(INFO) << "pps counter " << pps_counter;
            // LOG(INFO) << "time sec  " << time_sec;
            // LOG(INFO) << "time nsec  " << time_nsec;
            image_sync_ipc_server_ -> SetSyncInfo(time_sec, time_nsec, pps_counter);
        }
     }
}

}
}
}