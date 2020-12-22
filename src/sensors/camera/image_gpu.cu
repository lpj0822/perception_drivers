/*!
* \brief This file defines image gpu class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#include <holo/sensors/camera/image_gpu.h>

inline __device__ __host__ float clamp(float f, float a, float b)
{
    return fmaxf(a, fminf(f, b));
}


/* From RGB to YUV

   Y = 0.299R + 0.587G + 0.114B
   U = 0.492 (B-Y)
   V = 0.877 (R-Y)

   It can also be represented as:

   Y =  0.299R + 0.587G + 0.114B
   U = -0.147R - 0.289G + 0.436B
   V =  0.615R - 0.515G - 0.100B

   From YUV to RGB

   R = Y + 1.140V
   G = Y - 0.395U - 0.581V
   B = Y + 2.032U
 */

struct __align__(2) uchar6
{
   uchar3 a0, a1;
};
static __host__ __device__ __forceinline__ uchar6 make_uchar6(uchar3 a0, uchar3 a1)
{
   uchar6 val = {a0, a1};
   return val;
}

//__global__ void yuyvToRgb( uchar4* src, int srcAlignedWidth, uchar6* dst, int dstAlignedWidth, int width, int height )
__global__ void yuyvToRgb(uint8_t* yuyv, uint8_t* rgb, int srcAlignedWidth, int dstAlignedWidth, int width, int height )
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;

	if( x >= srcAlignedWidth || y >= height )
		return;

	if(y* srcAlignedWidth +x >= 1920/1080*2/4)
	{
	    //printf("there is a boundary %d\n", y* srcAlignedWidth +x);
	    //return;
	}
	const uchar4 macroPx = ((uchar4*)yuyv)[y * 1920/2 + x];// = src[y * srcAlignedWidth + x];
	

	// Y0 is the brightness of pixel 0, Y1 the brightness of pixel 1.
	// U0 and V0 is the color of both pixels.
	// UYVY [ U0 | Y0 | V0 | Y1 ] 
	// YUYV [ Y0 | U0 | Y1 | V0 ]
	const float y0 = macroPx.x;
	const float y1 = macroPx.z; 
	const float u = (macroPx.y) - 128.0f;
	const float v = (macroPx.w) - 128.0f;

	const float3 px0 = make_float3( y0 + 1.4065f * v,
				        y0 - 0.3455f * u - 0.7169f * v,
					y0 + 1.7790f * u);

	const float3 px1 = make_float3( y1 + 1.4065f * v,
					y1 - 0.3455f * u - 0.7169f * v,
					y1 + 1.7790f * u);

	((uchar6*)rgb)[y * 1920/2 + x] = make_uchar6( make_uchar3(clamp(px0.x, 0.0f, 255.0f), 
							       clamp(px0.y, 0.0f, 255.0f),
							       clamp(px0.z, 0.0f, 255.0f)),
                                                    make_uchar3(clamp(px1.x, 0.0f, 255.0f), 
							       clamp(px1.y, 0.0f, 255.0f),
							       clamp(px1.z, 0.0f, 255.0f)));
     
} 


void CudaYUYVToRGB(const int32_t& cols, const int32_t& rows, uint8_t* cuda_yuyv, uint8_t* cuda_rgb, uint8_t* yuyv, uint8_t* rgbb_image)
{
	cudaError_t cuda_status;

	cuda_status = cudaSetDevice(0);
	if(cuda_status != cudaSuccess)
	{
	    printf( "Device does not support cuda\n");
	    return;// cuda_status;
	}

	cuda_status = cudaMemcpy(cuda_yuyv, yuyv, cols * rows * 2, cudaMemcpyHostToDevice);
	if(cuda_status != cudaSuccess)
	{
	    printf( "Could not copy data to gpu\n");
	    return;// cuda_status;
	}

	dim3 block(8,8);
	dim3 grid(cols/2/8, rows/8);
        const int srcAlignedWidth = cols*rows*2 / sizeof(uchar4);	// normally would be uchar2, but we're doubling up pixels
	const int dstAlignedWidth = cols*rows*3 / sizeof(uchar6);	// normally would be uchar4 ^^^

	//printf("yuyvToRgba %zu %zu %i %i %i %i %i\n", width, height, (int)formatUYVY, srcAlignedWidth, dstAlignedWidth, grid.x, grid.y);

	//yuyvToRgb<<<grid, block>>>((uchar4*)cuda_yuyv, srcAlignedWidth, (uchar6*)cuda_rgb, dstAlignedWidth, cols, rows);
	yuyvToRgb<<<grid, block>>>(cuda_yuyv, cuda_rgb, srcAlignedWidth, dstAlignedWidth, cols, rows);

        cuda_status = cudaGetLastError();
        
        if(cuda_status != cudaSuccess)
	{
	    printf( "Yuyv to rgb error\n");
	    return;// cuda_status;
	}

        cuda_status = cudaDeviceSynchronize();
        if(cuda_status != cudaSuccess)
	{
	    printf("sychronization error\n");
	    return;// cuda_status;
	}

        cuda_status = cudaMemcpy(rgbb_image, cuda_rgb, cols*rows*3, cudaMemcpyDeviceToHost);
        if(cuda_status != cudaSuccess)
	{
	    printf("Could not copy data from gpu\n");
	    return;// cuda_status;
	}

        return;// cuda_status;
}