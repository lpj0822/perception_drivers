/*!
* \brief This file defines image capture class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#include <holo/sensors/camera/image_capture.h>
#include <glog/logging.h>
#include "raw2bmp.c"

namespace holo
{
namespace sensors
{
namespace camera
{

ImageCapture::ImageCapture(const std::string& yaml_file)
{
    if(!LoadYamlFile(yaml_file))
    {
        std::string error = "could not load yaml file";
        LOG(ERROR) << error;
        return;
    }

    rgbb_image_.image.create(camera_config_.rows, camera_config_.cols, CV_8UC3);
    rgbb_image_.encoding = Encoding::RGB;
    rgbb_image_.coord_id = camera_config_.coord;
    trigger_stamp_offset_ = AR023_STAMP_OFFSET;
    get_timestamp_ = false;
    last_pps_counter_ = 0;
    one_counter_ = 1;
    first_received_ = true;
    frame_counter_ = 0;
    pps_sec_ = 0;
    pps_nsec_ = 0;
    pps_counter_ = 0;    

    if (use_gpu_)
    {
        if(cudaMalloc((void**)&cuda_yuyv_, camera_config_.rows * camera_config_.cols* 2) != cudaSuccess)
        {
            LOG(ERROR) << "could not set cuda yuyv memory";
            return;
        } 

        if(cudaMalloc((void**)&cuda_rgb_, camera_config_.rows * camera_config_.cols* 3) != cudaSuccess)
        {
            LOG(ERROR) << "could not set cuda rgb memory";
            return;
        }     
    }

    if(camera_config_.trigger_mode)
    {
        image_sync_ipc_client_ = std::make_shared<ImageSyncIpc>(camera_config_.coord, 0, 0, false, true);        
    }

    capture_thread_ptr_ = std::shared_ptr<std::thread >(new std::thread(std::bind(
                           &ImageCapture::Start, this)));
}

ImageCapture::~ImageCapture()
{
   StopCapture();
   UninitDevice();
   CloseDevice();
   delete [] camera_config_.device_name;
}

bool ImageCapture::LoadYamlFile(const std::string& yaml_file)
{
    yaml::Node root_node;
    root_node = yaml::LoadFile(yaml_file);

    if (root_node["coord"])
    {
        camera_config_.coord = root_node["coord"].as<std::string>();
    }
    else
    {
        return false;
    }

    if (root_node["rows"])
    {
        camera_config_.rows = root_node["rows"].as<int32_t>();
    }
    else
    {
        return false;
    }

    if (root_node["cols"])
    {
        camera_config_.cols = root_node["cols"].as<int32_t>();
    }
    else
    {
        return false;
    }

    if (root_node["trigger_mode"])
    {
        camera_config_.trigger_mode = root_node["trigger_mode"].as<bool>();
    }
    else
    {
        return false;
    }

    if (root_node["device_name"])
    {
        std::string device_name = root_node["device_name"].as<std::string>();
        camera_config_.device_name = new char[device_name.length() + 1];
        strcpy(camera_config_.device_name, device_name.c_str());
    }
    else
    {
        return false;
    }

    if(root_node["pixel_format"])
    {
        camera_config_.pixel_format = static_cast<Encoding>(root_node["pixel_format"].as<int32_t>());
    }
    else
    {
        return false;
    }

    if(root_node["fps"])
    {
        camera_config_.fps = root_node["fps"].as<uint32_t>();
    }
    else
    {
        return false;
    }

    if(root_node["show_image"])
    {
        show_image_ = root_node["show_image"].as<bool>();
    }
    else
    {
        return false;
    }

    if(root_node["use_gpu"])
    {
        use_gpu_ = root_node["use_gpu"].as<bool>();
    }
    else
    {
        return false;
    }

    return true;
}

void ImageCapture::Start()
{

    if(!OpenDevice())
    {
        return;
    }
    
    if(!InitDevice())
    {
        return;
    }
    StartCapture();

}

bool ImageCapture::OpenDevice()
{
    struct stat st;
    LOG(INFO) << "open usb camera device";

    if (stat(camera_config_.device_name, &st) == -1)
    {
        LOG(ERROR) << "Cannot identify ' " << camera_config_.device_name  << "': " <<  errno << " " << strerror(errno);
        return false;
    }

    if (!S_ISCHR(st.st_mode))
    {
        LOG(ERROR) << camera_config_.device_name << " is not a character device";
        return false;
    }

    fd_ = open(camera_config_.device_name, O_RDWR | O_NONBLOCK, 0); // non-block

    if (fd_ == -1)
    {
        LOG(ERROR) << "Could not open device " << errno << " " <<  strerror(errno);
        return false;
    }

    return true;
}

bool ImageCapture::InitDevice()
{
    LOG(INFO) << "init device";
    struct v4l2_capability cap;   // some basic parameters
    struct v4l2_cropcap cropcap;  // this is used to set the capature ability of camera
    struct v4l2_crop crop;
    struct v4l2_format fmt;       // this is used to set the format of camera
    unsigned int min;

    if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) == -1)   // check if the device can be used as image capturing
    {
        if (EINVAL == errno)
        {
            LOG(ERROR) << "device is not V4L2 device";
            return false;
        }
        else
        {
            LOG(ERROR) << "VIDIOC_QUERYCAP";
            return false;
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        LOG(ERROR) << "device is not a video capture device";
        return false;
    }
    
    /* Select video input, video standard and tune here. */
    memset(&(cropcap), 0, sizeof(cropcap));

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(fd_, VIDIOC_CROPCAP, &cropcap) == 0)
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        memset(&(fmt), 0, sizeof(fmt));

        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = camera_config_.cols;//width_;
        fmt.fmt.pix.height      = camera_config_.rows;//height_;
        switch (static_cast<int32_t>(camera_config_.pixel_format))
        {
        case static_cast<int32_t>(Encoding::YUYV):
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
            //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
            break;
        default:
            LOG(ERROR) << "could not find the correct pixel format " << camera_config_.pixel_format ;
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
            break;
            //return false;
        }
        fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) == -1)
        {
            LOG(ERROR) << "VIDIOC_S_FMT";
            return false;
        }
        
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 1;

    if (fmt.fmt.pix.bytesperline < min)
    {
        fmt.fmt.pix.bytesperline = min;
    }

    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;

    if (fmt.fmt.pix.sizeimage < min)
    {
        fmt.fmt.pix.sizeimage = min;
    }

    if(camera_config_.trigger_mode)
    {
        SetTrigger(1);
    }
    else
    {
        SetTrigger(0);
    }

    if(!InitMmap())
    {
        return false;
    }

    return true;
}

bool ImageCapture::InitMmap()
{
    struct v4l2_requestbuffers req;

    memset(&(req), 0, sizeof(req));

    req.count = 4;                                     // first apply 4 sizes of spaces from device memory, but it depneds on the device, could be more or less
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;            // for video capture
    req.memory = V4L2_MEMORY_MMAP;                     // the memory is memory map from device memory

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) == -1)
    {
        if (EINVAL == errno)
        {
            LOG(ERROR) << "device dose not support memory maping";
            return false;
        }
        else
        {
            LOG(ERROR) << "VIDIOC_REQBUFS";
        }
    }

    if (req.count < 2)
    {
        LOG(ERROR) << "Insufficient buffer memory on device";
        return false;
    }

    buffers_ = (StreamBuffer*)calloc(req.count, sizeof(*buffers_));

    if (!buffers_)
    {
        LOG(ERROR) << "Out of memory\n";
        return false;
    }

    for (size_t buffer_idx = 0; buffer_idx < req.count; buffer_idx++)
    {
        struct v4l2_buffer buf;
        memset(&(buf), 0, sizeof(buf));

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = buffer_idx;

        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1)
        {
            LOG(ERROR) << "VIDIOC_QUERYBUF";
            return false;
        }

        buffers_[buffer_idx].length = buf.length;
        buffers_[buffer_idx].start = mmap(NULL /* start anywhere */,
                                          buf.length,
                                          PROT_READ | PROT_WRITE /* required */,
                                          MAP_SHARED /* recommended */,
                                          fd_, buf.m.offset);

        if (buffers_[buffer_idx].start == MAP_FAILED)
        {
            LOG(ERROR) << "mmap";
            return false;
        }
    }

    buffer_number_ = req.count;
    
    return true;
}

void ImageCapture::StartCapture()
{
    LOG(INFO) << "start device";
    unsigned int i;
    enum v4l2_buf_type type;

    for (i = 0; i < buffer_number_; i++)
    {
        struct v4l2_buffer buf;

        memset(&(buf), 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd_, VIDIOC_QBUF, &buf) == -1)
        {
             std::string error = "VIDIOC_QBUF";
             LOG(ERROR) << error;
             return;
        }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(fd_, VIDIOC_STREAMON, &type) == -1)
    {
        std::string error = "VIDIOC_STREAMON";
        LOG(ERROR) << error;
        return;
    }

    while(true)
    {
        while (true)
        {
            fd_set fds;
            struct timeval tv;
            int32_t r;

            FD_ZERO(&fds);
            FD_SET(fd_, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(fd_ + 1, &fds, NULL, NULL, &tv);

            if (r == -1)
            {
                if (errno == EINTR)
                {
                    continue;
                }

                std::string error = "select";
                LOG(ERROR) << error;
                return;
            }

            if (r == 0)
            {
                LOG(ERROR) << "select timeout";
            }

            // if read a frame from hardware break
            if (ReadFrame())
            {
                break;
            }
        }
    }
}

bool ImageCapture::ReadFrame()
{
    uint32_t test;
    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof(buf));

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_DQBUF, &buf) == -1)
    {
        switch (errno)
        {
            case EAGAIN:
                return false;

            case EIO:

                /* Could ignore EIO, see spec. */
                break;
                /* fall through */

            default:
                exit(EXIT_FAILURE);
        }
    }

    assert(buf.index < buffer_number_);

    Timestamp trigger_stamp;
    trigger_stamp.sec = 0;
    trigger_stamp.nsec = 0;
    Timestamp received_stamp = NowToGps();

    if(camera_config_.trigger_mode)
    {
        static uint32_t skip_frame = 0;
        if(skip_frame < 10)
        {
            skip_frame++;
            if(ioctl(fd_, VIDIOC_QBUF, &buf) == -1)
            {
                exit(EXIT_FAILURE);
            }
            return true;
        }
  
        if(one_counter_ == 1)
        {
            test = last_pps_counter_;
            image_sync_ipc_client_ -> GetSyncInfo(pps_sec_, pps_nsec_, pps_counter_);
            Timestamp a;
            a.sec = pps_sec_;
            a.nsec = 0;
            //LOG(INFO) << "get sec " << std::fixed << ToSec(UtcToGps(a));
            //LOG(INFO) << "get nsec " << std::fixed << pps_nsec_;

            if(first_received_)
            {
                LOG(INFO) << "pps counter1 " << pps_counter_;
                if(pps_counter_ - last_pps_counter_ == 1)
                {
                    get_timestamp_ = true;
                    first_received_ = false;
                    last_received_stamp_ = received_stamp;
                    LOG(INFO) << "find first frame!!!!!";
                    LOG(INFO) << "pps counter " << pps_counter_;
                    LOG(INFO) << "get sec " << std::fixed << ToSec(UtcToGps(a));
                    LOG(INFO) << "get nsec " << std::fixed << pps_nsec_;
                }
                else
                {
                    LOG(INFO) << "could not find first frame pps counter " << pps_counter_;
                    
                }
                last_pps_counter_ = pps_counter_;
            }
            else
            {
                if(pps_counter_ - last_pps_counter_ != 1)
                {
                    LOG(INFO) << "pps counter does not increase by one";
                    get_timestamp_ = false;
                }
                else
                {
                    get_timestamp_ = true;
                }
                last_pps_counter_ = pps_counter_;
            }
         
            if(!get_timestamp_)
            {
                last_received_stamp_ = received_stamp;
                LOG(INFO) << "Could not get timestamp from board";
                if(ioctl(fd_, VIDIOC_QBUF, &buf) == -1)
                {
                   exit(EXIT_FAILURE);
                }
                return true;
            }
        }

        float32_t delt_t = 1.0/camera_config_.fps;
        float64_t delt_received_t = ToSec(received_stamp) - ToSec(last_received_stamp_);
        last_received_stamp_ = received_stamp;

        if(delt_received_t > 1.5 * delt_t)
        {
            LOG(ERROR) << "The received delt t is too big which is " << delt_received_t;
            LOG(ERROR) << "Maybe there is a jump frame, skip  this frame";
            frame_counter_++;
            one_counter_ = 1;
            get_timestamp_ = false;
            if(ioctl(fd_, VIDIOC_QBUF, &buf) == -1)
            {
                   exit(EXIT_FAILURE);
            }
            return true;
        }

        trigger_stamp.sec = pps_sec_;
        trigger_stamp.nsec = pps_nsec_ + ((one_counter_) * delt_t + trigger_stamp_offset_) * 1e9;
        if(pps_sec_ > UTC_GPS_GAP)
        {
            trigger_stamp = UtcToGps(trigger_stamp);
        }
        
        float64_t time_diff = ToSec(trigger_stamp) - ToSec(last_trigger_stamp_);
        if(time_diff <= 0 || time_diff >= 0.051)
        {
            LOG(INFO) << "there is a time jump " << std::fixed << time_diff;
            LOG(INFO) << "one counter " << one_counter_;
            LOG(INFO) << "pps second " << pps_sec_;
            LOG(INFO) << "pps nsec " << pps_nsec_;
            LOG(INFO) << "pps counter " << pps_counter_ << " last counter " << test;
            LOG(INFO) << "current stamp " << std::fixed << ToSec(trigger_stamp);
            LOG(INFO) << "last stamp " << std::fixed << ToSec(last_trigger_stamp_);
            LOG(INFO) << "******** done ********** ";
        }
        last_trigger_stamp_ = trigger_stamp;
        if(one_counter_ < camera_config_.fps)
        {
            one_counter_++;
        }
        else if(one_counter_ == camera_config_.fps)
        {
            one_counter_ = 1;
        }
    }

    if(camera_config_.pixel_format == Encoding::RGB)
    {
       //LOG(INFO) << "get one image " << buf.bytesused;
        
        if(buf.bytesused != camera_config_.rows * camera_config_.cols* 2)
        {
            LOG(INFO) << "Could not convert yuyv to rgb";
            if (ioctl(fd_, VIDIOC_QBUF, &buf) == -1)
            {
                exit(EXIT_FAILURE);
            }
            return true;
        }
        else
        {
            if(use_gpu_)
            {
                CudaYUYVToRGB(camera_config_.cols, camera_config_.rows, cuda_yuyv_, cuda_rgb_, (uint8_t*)buffers_[buf.index].start, (uint8_t*)rgbb_image_.image.data);    
            }
            else
            {
                YUYVb2RGBb((uint8_t*)buffers_[buf.index].start, buf.bytesused, rgbb_image_);
            }
        }

        if(show_image_)
        {
            cv::Mat bgr;
            cv::cvtColor(rgbb_image_.image, bgr, CV_BGR2RGB);
            cv::imshow("driver", bgr);
            cvWaitKey(1);
        }

        rgbb_image_.timestamp = received_stamp;
        rgbb_image_.trigger_stamp = trigger_stamp;

        if(image_callback_)
        {
            image_callback_(rgbb_image_);
        }
    }

    if(camera_config_.pixel_format == Encoding::YUYV)
    {
        if(buf.bytesused != camera_config_.rows*camera_config_.cols*2)
        {
            if (ioctl(fd_, VIDIOC_QBUF, &buf) == -1)
            {
                exit(EXIT_FAILURE);
            }
            return true;
        }
        Image yuyv_image;
        yuyv_image.image.create(camera_config_.rows, camera_config_.cols, CV_8UC2);
        yuyv_image.encoding = Encoding::YUYV;
        yuyv_image.coord_id = camera_config_.coord;
        yuyv_image.timestamp = received_stamp;
        yuyv_image.trigger_stamp = trigger_stamp;
        memcpy(yuyv_image.image.data, (uint8_t*)buffers_[buf.index].start, buf.bytesused);

        if(image_callback_)
        {
            image_callback_(yuyv_image);
        }
    }
     

     if (ioctl(fd_, VIDIOC_QBUF, &buf) == -1)
     {
            exit(EXIT_FAILURE);
     }
        
     return true;      
}

void ImageCapture::StopCapture()
{

}

void ImageCapture::UninitDevice()
{
     for (size_t i = 0; i < buffer_number_; ++i)
        {
            if (munmap(buffers_[i].start, buffers_[i].length) == -1)
            {
                exit(EXIT_FAILURE);
            }
        }
}

void ImageCapture::CloseDevice()
{
    close(fd_);
}

void ImageCapture::SetTrigger(const int32_t& trigger_mode)
{
    uint8_t value[64] = {0};
    struct uvc_xu_control_query xu_query =
    {
        .unit       = 3, //has to be unit 3
        .selector   = 1, //TD
        .query      = UVC_SET_CUR,
        .size       = 4, //TD
        .data       = value,
    };

    xu_query.selector = CY_FX_UVC_XU_TRIGGER_ENABLE >> 8;
    xu_query.query = UVC_SET_CUR;
    xu_query.size = 2;

    switch (trigger_mode)
    {
    case (0):
        //Trigger mode disable,auto streaming
        value[0] = 0x00;
        value[1] = 0x00;
        break;
    case (1):
        //Trigger mode enable
        value[0] = 0x01;
        value[1] = 0x00;
        break;
    default:
        LOG(ERROR) << "incorrect trigger mode";
        break;
    }
    if (Xioctl(fd_, UVCIOC_CTRL_QUERY, &xu_query) != 0)
    {
        LOG(ERROR) << "could not trigger camera";
    }
    else
    {
        LOG(INFO) << "successfully trigger camera";
    }
    sleep(4);
}


int32_t ImageCapture::Xioctl(int32_t fd, int32_t id, void *arg)
{
    int32_t ret = 0;
    int32_t tries= 4;
    do
    {
        ret = ioctl(fd, id, arg);
    }
    while (tries-- );

    if (ret && (tries <= 0))
    {
        printf("ioctl (%i) retried %i times - giving up: %s)\n", id, 4, strerror(errno));
    } 

    return (ret);
}

}
}
}
