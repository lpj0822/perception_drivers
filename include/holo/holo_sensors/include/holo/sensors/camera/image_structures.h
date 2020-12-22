/*!
* \brief This file defines camera raw info data struct for objects attribute
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef HOLO_SENSORS_CAMERA_IMAGE_STRUCTURES_H
#define HOLO_SENSORS_CAMERA_IMAGE_STRUCTURES_H

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <holo/msg_header.h>

namespace holo
{
namespace sensors
{
namespace camera
{
    
enum Encoding
{
    BGR         = 0,
    RGB         = 1,
    YUV         = 2,
    GRAYSCALE   = 3,
    YUYV        = 4,
};

/**
 * @brief      yuy data for each pixel
 */
struct YUVb
{
    uint8_t y;
    uint8_t u;
    uint8_t v;
};

/**
 * @brief      rgb data for each pixel
 */
struct RGBb
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

/**
 * @brief      stream buffer from hardware
 */
struct StreamBuffer
{
    void*   start;
    size_t  length;
};

enum CameraModel
{
    Entron    =    1,      // the rolling shutter camera 1920*1080
    UNKNOWN   =    255,    //
};

/**
 * @brief      camera config parameters
 */
struct CameraConfig
{
    CameraModel model;
    std::string coord;
    int32_t     rows;
    int32_t     cols;
    bool        trigger_mode;
    char*       device_name;
    Encoding pixel_format;
    uint32_t    fps;
    std::string ipc_file;
};

struct Image
{
    Timestamp timestamp;
    std::string coord_id;
    uint32_t rows;
    uint32_t cols;
    Timestamp trigger_stamp;
    cv::Mat image;
    enum Encoding encoding;
};

}
}
}

#endif
