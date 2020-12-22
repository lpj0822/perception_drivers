/*!
* \brief This file defines image sync ipc class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#include <holo/sensors/camera/image_sync_ipc.h>
#include <glog/logging.h>

namespace holo
{
namespace sensors
{
namespace camera
{

ImageSyncIpc::ImageSyncIpc(const std::string& camera_coord, const int32_t& rows, const int32_t& cols, bool is_server, bool sync_ipc)
{
    std::string ipc_file = "/tmp/camera_ipc/";

    if(sync_ipc)
    {
        ipc_file = ipc_file + std::string("camera_sync");
    }
    else
    {
        if(camera_coord == "CAMERA_FRONT_CENTER")
        {
            ipc_file = ipc_file + std::string("camera_front_center");
        }
        else if(camera_coord == "CAMERA_FRONT_LEFT")
        {
            ipc_file = ipc_file + std::string("camera_front_left");
        }
        else if(camera_coord == "CAMERA_FRONT_RIGHT")
        {
            ipc_file = ipc_file + std::string("camera_front_right");
        }
        else if(camera_coord == "CAMERA_REAR_CENTER")
        {
            ipc_file = ipc_file + std::string("camera_rear_center");   
        }
        else if(camera_coord == "CAMERA_FRONT_CENTER_GS")
        {
            ipc_file = ipc_file + std::string("camera_front_center_gs");
        }
        else
        {
            std::string error = "could not find corresponding coord";
            LOG(ERROR) << error;
            return;
        }
    }

    LOG(INFO) << "open the shared memory " << ipc_file.c_str();
    
    int32_t length = ipc_file.length();
    shared_file_name_ = new char[length + 1];
    strcpy(shared_file_name_, ipc_file.c_str());
    
    // image_size_ = rows * cols * 3;
    // cols_ = cols;
    // rows_ = rows;
    sync_ipc_ = sync_ipc;
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
        size_t total_size;
        if(sync_ipc)
        {
            total_size = CAMERA_SYNC_SIZE;
        }
        else
        {
            //total_size = image_size_ + sizeof(MsgHeader) + sizeof(TimeStamp) + 1;
        }

        if(!InitIpcServer(total_size))
        {
            LOG(ERROR) << "could not init ipc server";
            return;
        }
    }
}

bool ImageSyncIpc::InitIpcServer(const size_t& total_size)
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
    uint32_t tmp_value = 0;
    memcpy((uint8_t*)shared_mem_address_, &tmp_value, 4);
    memcpy((uint8_t*)shared_mem_address_+4, &tmp_value, 4);
    memcpy((uint8_t*)shared_mem_address_+8, &tmp_value, 4);
    return true;
}

bool ImageSyncIpc::InitIpcClient()
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

bool ImageSyncIpc::GetUse()
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

bool ImageSyncIpc::ReleaseUse()
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

bool ImageSyncIpc::GetSyncInfo(uint32_t& sec, uint32_t& nsec, uint32_t& pps_counter)
{
	if(!GetUse())
	{
		return false;
	}
	memcpy(&sec, (uint8_t*)shared_mem_address_, 4);
    memcpy(&nsec, (uint8_t*)shared_mem_address_+4, 4);
	memcpy(&pps_counter, (uint8_t*)shared_mem_address_+8, 4);
	ReleaseUse();
	return true;
}

bool ImageSyncIpc::SetSyncInfo(const uint32_t& sec, const uint32_t& nsec, const uint32_t& pps_counter)
{
	if(!GetUse())
	{
		return false;
	}
	memcpy((uint8_t*)shared_mem_address_, &sec, 4);
    memcpy((uint8_t*)shared_mem_address_+4, &nsec, 4);
	memcpy((uint8_t*)shared_mem_address_+8, &pps_counter, 4);
	ReleaseUse();
	return true;
}


}
}
}