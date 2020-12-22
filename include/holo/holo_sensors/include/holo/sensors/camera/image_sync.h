/*!
* \brief This file defines image sync header
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef HOLO_SENSORS_CAMERA_IMAGE_SYNC_H
#define HOLO_SENSORS_CAMERA_IMAGE_SYNC_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>
#include <holo/yaml.h>
#include <holo/sensors/camera/image_sync_ipc.h>

namespace holo
{
namespace sensors
{
namespace camera
{

class ImageSync
{

public:
    ImageSync(const std::string& yaml_file);

private:
    bool LoadParameters(const std::string& yaml_file);
    bool SetUdpConnection();
    void GetSyncInfo();

    std::shared_ptr<std::thread> udp_thread_ptr_;
    std::shared_ptr<ImageSyncIpc> image_sync_ipc_server_;

    std::string ip_;
    uint32_t port_;
    int32_t sock_;
    struct sockaddr_in recv_addr_;
};

}
}
}

#endif