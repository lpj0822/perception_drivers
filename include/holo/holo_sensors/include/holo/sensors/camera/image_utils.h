/*!
* \brief This file defines image utils header
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef HOLO_SENSORS_CAMERA_IMAGE_UTILS_H
#define HOLO_SENSORS_CAMERA_IMAGE_UTILS_H

#include <holo/yaml.h>
#include <holo/sensors/camera/image_structures.h>

namespace holo
{
namespace sensors
{
namespace camera
{

/**
 * @brief      yuv pixel to rgb pixel
 *
 * @param[in]  yuv   The yuv pixel data
 * @param      rgb   The rgb pixel data
 */
void YUVb2RGBb(const YUVb& yuv, RGBb& rgb);



/**
 * @brief      convert yuyv image to rgb image
 *
 * @param[in]  yuyv  The buffer of yuyv image
 * @param[in]  size  The size of image
 * @param[out] rgb   The rgb image
 * @return     ture  if the conversion is done otherwise return false
 */
bool YUYVb2RGBb(uint8_t* yuyv, const size_t& size, Image& rgb);


}
}
}

#endif
