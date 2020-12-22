/*!
* \brief This file defines image ipc class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#include <holo/sensors/camera/image_ipc.h>
#include <glog/logging.h>

namespace holo
{
namespace sensors
{
namespace camera
{

ImageIpc::ImageIpc(const Coord& camera_coord, const int32_t& rows, const int32_t& cols, bool is_server)
{
    std::string ipc_file = SENSORS_PARAM_DIR + std::string("camera_ipc/");

    if(camera_coord == Coord::Value::CAMERA_FRONT_CENTER)
    {
        ipc_file = ipc_file + std::string("camera_front_center");
    }
    else if(camera_coord == Coord::Value::CAMERA_FRONT_LEFT)
    {
        ipc_file = ipc_file + std::string("camera_front_left");
    }
    else if(camera_coord == Coord::Value::CAMERA_FRONT_RIGHT)
    {
        ipc_file = ipc_file + std::string("camera_front_right");
    }
    else
    {
        std::string error = "could not find corresponding coord";
        throw HoloException(error);
        return;
    }
    
    LOG(INFO) << "open the shared memory " << ipc_file.c_str();
    
    int32_t length = ipc_file.length();
    shared_file_name_ = new char[length + 1];
    strcpy(shared_file_name_, ipc_file.c_str());
    
    image_size_ = rows * cols * 3;
    cols_ = cols;
    rows_ = rows;
    if(!is_server)
    {
        LOG(INFO) << "init ipc client";
        is_server_ = false;
        if(!InitIpcClient())
        {
            LOG(INFO) << "could not init ipc client";
            return;
        }
    }
    else
    {
        LOG(INFO) << "init ipc server";
        is_server_ = true;
        size_t total_size = image_size_ + sizeof(MsgHeader) + sizeof(TimeStamp) + 1;
        if(!InitIpcServer(total_size))
        {
            LOG(ERROR) << "could not init ipc server";
            return;
        }
    }
}

ImageIpc::~ImageIpc()
{

}

bool ImageIpc::InitIpcServer(const size_t& total_size)
{
    semkey_ = ftok(shared_file_name_, 1);
    if (semkey_ == -1)
    {
        LOG(ERROR) << "could not build sem file";
        LOG(ERROR) << strerror(errno) << "errno:" << errno;
        return false;
    }

    shmkey_ = ftok(shared_file_name_, 1);
    if (shmkey_ == -1)
    {
        LOG(ERROR) << "could not build shm file";
        return false;
    }

    semid_ = semget(semkey_, 1, 0666 | IPC_CREAT);
    if (semid_ == -1)
    {
        LOG(ERROR) << "could not get sem id";
        LOG(ERROR) << strerror(errno) << "errno:" << errno;
        return false;
    }

    union semun sem_args;
    unsigned short array[1] = {1};
    sem_args.array = array;
    semctl(semid_, 0, SETALL, sem_args);
    shmid_ = shmget(shmkey_, total_size, 0666 | IPC_CREAT);
    if (shmid_ == -1)
    {
        LOG(ERROR) << "could not get shm id";
        LOG(ERROR) << strerror(errno) << "errno:" << errno;
        return false;
    }

    shared_mem_address_ = shmat(shmid_, NULL, 0);
    if (shared_mem_address_ == NULL)
    {
        LOG(ERROR) << "could not bind shared memory address";
        return false;
    }
    
    return true;
}

bool ImageIpc::InitIpcClient()
{
    semkey_ = ftok(shared_file_name_, 1);
    if (semkey_ == -1)
    {
        LOG(ERROR) << "could not build sem file";
        return false;
    }

    shmkey_ = ftok(shared_file_name_, 1);
    if (shmkey_ == -1)
    {
        LOG(ERROR) << "could not build shm file";
        return false;
    }
    
    while(true)
    {
        semid_ = semget(semkey_, 1, 0666);
        if (semid_ == -1)
        {
            LOG(ERROR) << "could not get sem id";
            LOG(ERROR) << strerror(errno) << "errno:" << errno;
            sleep(1);
        }
        else
        {
            break;
        }
    }

    shmid_ = shmget(shmkey_, 0, 0666);
    if (shmid_ == -1)
    {
        LOG(ERROR) << "could not get shmid";
        return false;
    }

    shared_mem_address_ = shmat(shmid_, NULL, 0);
    if (shared_mem_address_ == NULL)
    {
        LOG(ERROR) << "could not bind shared memory address";
        return false;
    }
    
    return true;
}

bool ImageIpc::GetUse()
{
    operation_.sem_num = 0;
    operation_.sem_op = -1;
    operation_.sem_flg = 0;
    if (semop(semid_, &operation_, 1) == -1)
    {
        LOG(ERROR) << "could not set operation for sem when get use";
        LOG(ERROR) << strerror(errno) << " errno:" << errno;
        return false;
    }
    return true;
}

bool ImageIpc::ReleaseUse()
{
    operation_.sem_num = 0;
    operation_.sem_op = 1;
    operation_.sem_flg = 0;
    if (semop(semid_, &operation_, 1) == -1)
    {
        LOG(INFO) << "could not set operation for sem when release use";
        LOG(INFO) << strerror(errno) << " errno:" << errno;
        return false;
    }
    return true;
}

bool ImageIpc::SetImageData(const vision::Image& image)
{
    if(!image.image().isContinuous())
    {
        LOG(ERROR) << "the memory for cv mat is not continouous";
        return false;
    }
    
    size_t image_size = image.image().total() * image.image().elemSize();
    
    if(image_size != image_size_)
    {
        LOG(ERROR) << "the image size is not equal";
        return false;
    }
    
    if(!GetUse())
    {
        return false;
    }
    
    uint32_t offset = 0;
    memcpy((uint8_t*)shared_mem_address_, &image.header(), sizeof(MsgHeader));
    offset += sizeof(MsgHeader);
    memcpy((uint8_t*)shared_mem_address_+offset, &image.trigger_stamp(), sizeof(TimeStamp));
    offset += sizeof(TimeStamp);
    memcpy((uint8_t*)shared_mem_address_+offset, image.image().data, image_size);
    offset += image_size;
    memcpy((uint8_t*)shared_mem_address_+offset, &image.encoding(), 1);
    
    ReleaseUse();
    
    return true;
}

bool ImageIpc::GetImageData(vision::Image& image)
{
    image.image().create(rows_, cols_, CV_8UC3);
    if(!GetUse())
    {
        return false;
    }
    uint32_t offset = 0;
    memcpy(&image.header(), (uint8_t*)shared_mem_address_, sizeof(MsgHeader));
    offset += sizeof(MsgHeader);
    memcpy(&image.trigger_stamp(), (uint8_t*)shared_mem_address_+offset, sizeof(TimeStamp));
    offset += sizeof(TimeStamp);
    memcpy(image.image().data, (uint8_t*)shared_mem_address_+offset, image_size_);
    offset += image_size_;
    memcpy(&image.encoding(), (uint8_t*)shared_mem_address_+offset, 1);

    ReleaseUse();
    return true;
}

}
}
}