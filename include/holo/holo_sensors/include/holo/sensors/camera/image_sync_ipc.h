/*!
* \brief This file defines image sync ipc header
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#ifndef HOLO_SENSORS_CAMERA_IMAGE_SYNC_IPC_H
#define HOLO_SENSORS_CAMERA_IMAGE_SYNC_IPC_H

#include <stdio.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <iostream>

#include <holo/yaml.h>

namespace holo
{
namespace sensors
{
namespace camera
{

#define CAMERA_SYNC_SIZE     12

class ImageSyncIpc
{

public:
    ImageSyncIpc(const std::string& camera_coord, const int32_t& rows, const int32_t& cols, bool is_server=false, bool sync_ipc = false);

    bool GetSyncInfo(uint32_t& sec, uint32_t& nsec, uint32_t& pps_counter);

    bool SetSyncInfo(const uint32_t& sec, const uint32_t& nsec, const uint32_t& pps_counter);

private:
    /**
    * \brief initialize of ipc server usually the driver would make this as server
    *
    * @return true if the server is successfully built otherwise return false
    */
    bool InitIpcServer(const size_t& total_size);
    /**
    * \brief initialize of ipc client
    *
    * @return true if the client is successfully built otherwise return false
    */
    
    bool InitIpcClient();
    /**
    * \brief get access to shared memory
    *
    * @return true if the shared is ready to use otherwise return false
    */
    bool GetUse();
    /**
    * \brief release the shared memory then the other process could visit the shared memory
    *
    * @return true if the release is done otherwise return false
    */
    bool ReleaseUse();

    /// the semaphore id and shared memory id 
    int32_t semid_, shmid_;
    /// the semaphore key and shared memory key
    key_t semkey_, shmkey_;
    /// the file name of shared memory
    char* shared_file_name_;
    /// indicates if the process is server
    bool is_server_;
    /// the address of shared memory
    void* shared_mem_address_;
    struct sembuf operation_;
    struct shmid_ds shmid_struct_;
    union semun
    {
        int32_t val;
        struct semid_ds* buf;
        unsigned short* array;
    };

    std::string ipc_file_;
    bool sync_ipc_;
};

}
}
}

#endif