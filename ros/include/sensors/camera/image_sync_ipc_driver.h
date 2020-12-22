/*!
 *  \brief This file defines image sync ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLOMATIC_IMAGE_SYNC_IPC_DRIVER_H_
#define HOLOMATIC_IMAGE_SYNC_IPC_DRIVER_H_

#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <holo/sensors/camera/image_sync.h>
#include "holomatic/CameraSync.h"
#include <ros/ros.h>

namespace holomatic
{
namespace camera
{

class ImageIpcSync
{

public:
	ImageIpcSync(ros::NodeHandle& nh);

private:
	bool GetParameters(ros::NodeHandle& nh);
	void ImageSyncHandler(uint32_t& pps_counter, uint32_t& sync_sec, uint32_t& sync_nsec);

	std::string yaml_file_;
	holomatic::CameraSync camera_sync_;
	ros::Publisher pub_camera_sync_;

    std::shared_ptr<holo::sensors::camera::ImageSync> image_sync_;
};

}
}
#endif