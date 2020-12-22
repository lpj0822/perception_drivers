/*!
 *  \brief
 *  \author liwenjun (liwenjun@holomatic.com)
 *  \date 2017-12-08
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice.
 */

#include <glog/logging.h>
#include <sensors/camera/image_sync_ipc_driver.h>

namespace holomatic
{
namespace camera
{
ImageIpcSync::ImageIpcSync(ros::NodeHandle& nh)
{
    if(!GetParameters(nh))
    {
        ROS_ERROR("ImageCapture config file not found");
    }

    pub_camera_sync_ = nh.advertise<holomatic::CameraSync>("/camera/camera_sync", 1);

    image_sync_ = std::make_shared<holo::sensors::camera::ImageSync>(yaml_file_);
}

bool ImageIpcSync::GetParameters(ros::NodeHandle& nh)
{
    bool pass = true;
    pass &= nh.getParam("yaml_file", yaml_file_);

    return pass;
}

void ImageIpcSync::ImageSyncHandler(uint32_t& pps_counter, uint32_t& sync_sec, uint32_t& sync_nsec)
{
    camera_sync_.pps_counter = pps_counter;
    camera_sync_.sync_sec = sync_sec;
    camera_sync_.sync_nsec = sync_nsec;
    pub_camera_sync_.publish(camera_sync_);
}

}
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "camera_sync_driver");

    ros::NodeHandle nh("~");

    holomatic::camera::ImageIpcSync node(nh);
    
    ros::spin();
    
    return 0;
}
