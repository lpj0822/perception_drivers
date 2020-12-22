/*!
* \brief This file defines image utils for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#include <holo/sensors/camera/image_utils.h>
#include <glog/logging.h>

namespace holo
{
namespace sensors
{
namespace camera
{


void YUVb2RGBb(const YUVb& yuv, RGBb& rgb)
{
    int16_t r = yuv.y + 1.370705 * (yuv.v-128);
    int16_t g = yuv.y - 0.698001 * (yuv.v-128) - 0.337633 * (yuv.u - 128);
    int16_t b = yuv.y + 1.732446 * (yuv.u-128);

    rgb.r = r < 0 ? 0 : r > 255 ? 255 : r;
    rgb.g = g < 0 ? 0 : g > 255 ? 255 : g;
    rgb.b = b < 0 ? 0 : b > 255 ? 255 : b;
}

bool YUYVb2RGBb(uint8_t* yuyv, const size_t& size, Image& rgb)
{
    if(size == 0)
    {
        LOG(ERROR) << "the size of yuyv is zero";
        return false;
    }
    
    size_t rgb_size = rgb.image.total() * rgb.image.elemSize();
    if(rgb_size != 1.5 * size)
    {
        LOG(ERROR) << "the size of rgb is not 1.5 times of yuyv";
        return false;
    }
    
    size_t rgbb_index = 0;
    for (size_t i = 0; i != size; i = i + 4)
    {
        int32_t u = yuyv[i+1] - 128;
        int32_t v = yuyv[i+3] - 128;

        int32_t r = yuyv[i] + 1.4065 * v;
        int32_t g = yuyv[i] - 0.3455 * v - 0.7169 * u;
        int32_t b = yuyv[i] + 1.7790 * u;

        *(rgb.image.data+rgbb_index) = r < 0 ? 0 : r > 255 ? 255 : r;
        *(rgb.image.data+rgbb_index+1) = g < 0 ? 0 : g > 255 ? 255 : g;
        *(rgb.image.data+rgbb_index+2) = b < 0 ? 0 : b > 255 ? 255 : b;

        r = yuyv[i+2] + 1.4065 * v;
        g = yuyv[i+2] - 0.3455 * v - 0.7169 * u;
        b = yuyv[i+2] + 1.7790 * u;

        *(rgb.image.data+rgbb_index+3) = r < 0 ? 0 : r > 255 ? 255 : r;
        *(rgb.image.data+rgbb_index+4) = g < 0 ? 0 : g > 255 ? 255 : g;
        *(rgb.image.data+rgbb_index+5) = b < 0 ? 0 : b > 255 ? 255 : b;

        rgbb_index = rgbb_index + 6;

    }

    return true;
}



}
}
}