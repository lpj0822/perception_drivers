/*!
* \brief This file defines image capture header
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef HOLO_SENSORS_CAMERA_IMAGE_CAPTURE_H
#define HOLO_SENSORS_CAMERA_IMAGE_CAPTURE_H

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <iostream>
#include <thread>
#include <memory>
#include <mutex>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#include <holo/yaml.h>
#include <holo/sensors/camera/image_gpu.h>
#include <holo/sensors/camera/image_structures.h>
#include <holo/sensors/camera/image_utils.h>
#include <holo/sensors/camera/image_sync_ipc.h>
//#include <holo/sensors/camera/image_gpu.h>

namespace holo
{
namespace sensors
{
namespace camera
{
/**
 * @brief      The memory address for USB camera
 *
 * @param      0x0b00: set if the camera is in trigger mode
 * @param      0x0a00: set the trigger delay
 *
 */
#define CY_FX_UVC_XU_TRIGGER_ENABLE (0x0b00)
#define CY_FX_UVC_XU_TRIGGER_DELAY_TIME (0x0a00)

#define UTC_GPS_GAP               315964800
// for AR023 camera because it is a rolling shutter camera and the image needs
// some time to transmit, these is an offset for trigger stamp. where 28 is the
// free run frequency of the sensor
#define AR023_STAMP_OFFSET        -1.0/28

/**
 * @brief      Class for image capture.
 */
class ImageCapture
{

public:
    /**
     * The callback for image counter
     */
    typedef std::function<void(Image&)> ImageCallback;
    
    /**
     * @brief      constructor
     *
     * @param[in]  yaml_file  The config yaml file
     */
    ImageCapture(const std::string& yaml_file);


    /**
     * @brief      constructor
     */
    ImageCapture() = delete; 

    /**
     * @brief      Destroys the object.
     */
    ~ImageCapture();

    
    /**
     * @brief      Sets the image counter callback.
     *
     * @param[in]  handler  The handler
     */
    void SetImageCallback(ImageCallback handler)
    {
        image_callback_ = handler;
    }

private:

    /**
     * @brief      Loads config from yaml file.
     *
     * @param[in]  yaml_file  The yaml file
     *
     * @return     true if the yaml file is loaded otherwise return false
     */
    bool LoadYamlFile(const std::string& yaml_file);

    /**
     * @brief      Start to capture image
     */
    void Start();

    /**
     * @brief      Opens a device.
     *
     * @return     true if the device is opened otherwise return false
     */
    bool OpenDevice();



    /**
     * @brief      init the device
     *
     * @return     true if the device is init successfully otherwise retuan false
     */
    bool InitDevice();



    /**
     * @brief      Init memory map method
     *
     * @return     true it memory map method is OK otherwise return false
     */
    bool InitMmap();



    /**
     * @brief      Starts camera capture.
     */
    void StartCapture();



    /**
     * @brief      Reads a frame.
     *
     * @return     true if a frame is read from device otherwise return false
     */
    bool ReadFrame();
    


    /**
     * @brief      Stops capture.
     */
    void StopCapture();



    /**
     * @brief      uninit the device
     */
    void UninitDevice();



    /**
     * @brief      Closes device.
     */
    void CloseDevice();
    
    /**
     * @brief      Set the camera trigger mode.
     *
     * @param[in]  trigger_mode  1 if the camera is set to use external trigger 0 the camera would be in free run mode
     */
    void SetTrigger(const int32_t& trigger_mode);

    // video releated
    /**
    * camera configuration parameters
    */
    CameraConfig camera_config_;
    /**
     * file description
     */
    int32_t fd_;
    /**
     * buffer number of input frame
     */
    size_t buffer_number_;
    /**
     * the buffer of input frame
     */
    StreamBuffer* buffers_;
    /**
     * the input image and make it to rgb format
     */
    Image rgbb_image_;
    
    
    /**
     * thread for image capture
     */
    std::shared_ptr<std::thread> capture_thread_ptr_;
    
    /**
     * callback for image
     */
    ImageCallback image_callback_;

    std::shared_ptr<ImageSyncIpc> image_sync_ipc_client_;
    
     /**
      * @brief      differ from ioctl this furntion would communicate with
      *             camera multi times
      *
      * @param[in]  fd    file description
      * @param[in]  id    the input id
      * @param      arg   The argument
      *
      * @return     0 the action is OK <0 failed
      */
    int32_t Xioctl(int32_t fd, int32_t id, void *arg);   

    /**
     * set if the driver is going to show images
     */
    bool show_image_;

    bool use_gpu_;

    uint8_t *cuda_yuyv_, *cuda_rgb_;

    /**
     * indicates if timestamp from external board is received and if first pps signal is received
     */
    bool get_timestamp_, first_received_;
    /**
     * some variables to record the information
     */
    int32_t one_counter_;
    uint32_t last_pps_counter_, frame_counter_;
    uint32_t pps_sec_, pps_nsec_, pps_counter_;
    Timestamp last_trigger_stamp_, last_received_stamp_;
    float32_t trigger_stamp_offset_;    

};

}
}
}

#endif
