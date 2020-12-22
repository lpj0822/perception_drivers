/*!
* \brief This file defines image gpu header
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef _HOLO_IMAGE_GPU_H
#define _HOLO_IMAGE_GPU_H


#include <cuda_runtime.h>
#include <cuda.h>

#include <iostream>

void CudaYUYVToRGB(const int32_t& cols, const int32_t& rows, uint8_t* cuda_yuyv, uint8_t* cuda_rgb, uint8_t* yuyv, uint8_t* rgbb_image);


#endif