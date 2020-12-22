/*!
 *  \brief camera ros driver
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice.
 */

#include <sensors/camera/camera_driver.h>

namespace holomatic
{
namespace camera
{

CameraDriver::CameraDriver(ros::NodeHandle& nh)
{
    if(!GetParameters(nh))
    {
    	ROS_ERROR("Could not load parameters for camera driver");
    	return;
    }

    std::string topic_name;

    if (coord_ == "CAMERA01")
    {
        topic_name = "/camera/01";
    }
    else if (coord_ == "CAMERA02")
    {
        topic_name = "/camera/02";
    }
    else if (coord_ == "CAMERA03")
    {
        topic_name = "/camera/03";
    }
    else if (coord_ == "CAMERA04")
    {
        topic_name = "/camera/04";
    }
   else if (coord_ == "CAMERA05")
    {
        topic_name = "/camera/05";
    }
    else
    {
        /* TODO add other coord */
    }


    pub_raw_ = nh.advertise<sensor_msgs::Image>(topic_name, 1);

    image_capture_ = std::make_shared<holo::sensors::camera::ImageCapture>(yaml_file_);

    image_capture_ -> SetImageCallback(std::bind(&CameraDriver::ImageHandler, this, std::placeholders::_1));
}

bool CameraDriver::GetParameters(ros::NodeHandle& nh)
{
	bool pass = true;
    pass &= nh.getParam("yaml_file", yaml_file_);
    pass &= nh.getParam("coord", coord_);
    pass &= nh.getParam("pixel_format", pixel_format_);

    return pass;
}

void CameraDriver::ImageHandler(holo::sensors::camera::Image& input)
{
    raw_image_.header.frame_id = input.coord_id;
    raw_image_.header.stamp.sec = input.trigger_stamp.sec;
    raw_image_.header.stamp.nsec = input.trigger_stamp.nsec;
    raw_image_.encoding = sensor_msgs::image_encodings::RGB8;

    //! RGB format
    if (pixel_format_ == 1)
    {
        raw_image_.data.resize(1920*1020*3);
    }
    //! YUYV format
    else if (pixel_format_ == 2)
    {
        raw_image_.data.resize(1920*1020*2);
    }

    raw_image_.is_bigendian = false;
    raw_image_.height = input.image.rows;
    raw_image_.width = input.image.cols;
    raw_image_.step = input.image.step;

    memcpy(&raw_image_.data.at(0), input.image.data, raw_image_.step * raw_image_.height);

    std::cout << "stamp:" << std::setprecision(32) << raw_image_.header.stamp.toNSec() / 1000ull / 1000000.0 << std::endl;

    pub_raw_.publish(raw_image_);
}

}
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "camera_driver");

    ros::NodeHandle nh("~");

    holomatic::camera::CameraDriver node(nh);
    
    ros::spin();
    
    return 0;
}
