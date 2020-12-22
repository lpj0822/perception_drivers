/*!
 *  \brief This file defines camera ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLOMATIC_CAMERA_DRIVER_H_
#define HOLOMATIC_CAMERA_DRIVER_H_

#include <holo/sensors/camera/image_capture.h>
#include "holomatic/CameraSync.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>

namespace holomatic
{
namespace camera
{

class CameraDriver
{
public:
    CameraDriver(ros::NodeHandle& nh);

private:
    bool GetParameters(ros::NodeHandle& nh);
    void ImageHandler(holo::sensors::camera::Image& input);

    std::shared_ptr<holo::sensors::camera::ImageCapture> image_capture_;
    std::string yaml_file_;
    std::string coord_;
    int pixel_format_;

    ros::Publisher pub_raw_;
    sensor_msgs::Image raw_image_;
};

}
}

#endif