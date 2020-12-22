/*!
* \brief This file defines Contiradar class for clusters parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLO_SENSORS_CONTIRADAR_CLUSTER_H_
#define HOLO_SENSORS_CONTIRADAR_CLUSTER_H_

namespace holo
{
namespace sensors
{
namespace contiradar
{

//! @brief fetch object signal of object[index].partno.signal from 'data'
//!        and store to local
#define CONTIRADAR_STORE_CLUSTER_SIGNAL_ID(partno, type, data, location)   do {                                          \
temp_cluster_[location].part##partno.ID =                                                                                \
GET_Cluster_##partno##_##type##_Cluster_ID(data);                                                                        \
} while(0)

#define CONTIRADAR_STORE_CLUSTER_SIGNAL_PART1(index1, partno, type, signal, data, location)   do {                       \
local_cluster_[location][index1].part##partno.signal =                                                                   \
GET_Cluster_##partno##_##type##_Cluster_##signal(data);                                                                  \
} while(0)                                                              

#define CONTIRADAR_STORE_CLUSTER_SIGNAL_PART2(index1, partno, type, signal, data, location)   do {                       \
local_cluster_[location][index1].part##partno.signal =                                                                   \
GET_Cluster_##partno##_##type##_Cluster_##signal(data);                                                                  \
} while(0)

//! @brief calculate physical value of object[index].partno.signal
#define CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(index1, partno, type, signal, location)                                       \
CALC_Cluster_##partno##_##type##_Cluster_##signal(local_cluster_[location][index1].part##partno.signal, 1.0)                          

//! @brief fetch local data and convert these data to standard format one,
//!        and push them into array for transmission
#define CONTIRADAR_CONVERT_PUSH_CLUSTER(index1, location)    do {                                                        \
    /* check if all of part1~3 are received */                                                                           \
    Ars048Obstacle obs;                                                                                                  \
    /*if(local_cluster_[location][index1].part1_valid == 1 && local_cluster_[location][index1].part2_valid == 1 )      */\
    {                                                                                                                    \
        obs.timestamp = local_cluster_[location][index].timestamp;                                                       \
        obs.coord = local_cluster_[location][index].coord;                                                               \
        obs.obj_id  = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                                \
            index, 1, General, ID, location);                                                                            \
        obs.obj_dist_long = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                          \
            index, 1, General, DistLong, location);                                                                      \
        obs.obj_dist_lat = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                           \
            index, 1, General, DistLat, location);                                                                       \
        obs.obj_vrel_long = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                          \
            index, 1, General, VrelLong, location);                                                                      \
        obs.obj_vrel_lat = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                           \
            index, 1, General, VrelLat, location);                                                                       \
        obs.obj_dyn_prop = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                           \
            index, 1, General, DynProp, location);                                                                       \
        obs.obj_rcs = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                                \
            index, 1, General, RCS, location);                                                                           \
        obs.obj_dist_long_rms = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                      \
            index, 2, Quality, DistLong_rms, location);                                                                  \
        obs.obj_dist_lat_rms = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                       \
            index, 2, Quality, DistLat_rms, location);                                                                   \
        obs.obj_vrel_long_rms = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                      \
            index, 2, Quality, VrelLong_rms, location);                                                                  \
        obs.obj_vrel_lat_rms = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                       \
            index, 2, Quality, VrelLat_rms, location);                                                                   \
        obs.pdh0 = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                                   \
            index, 2, Quality, Pdh0, location);                                                                          \
        obs.ambig_state = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                            \
            index, 2, Quality, AmbigState, location);                                                                    \
        obs.invalid_state = CONTIRADAR_LOAD_CLUSTER_SIGNAL_PHY(                                                          \
            index, 2, Quality, InvalidState, location);                                                                  \
        obs.obj_arel_long_rms = 0;                                                                                       \
        obs.obj_arel_lat_rms = 0;                                                                                        \
        obs.obj_orientation_rms = 0;                                                                                     \
        obs.obj_meas_state = 0;                                                                                          \
        obs.obj_prob_of_exist = 0;                                                                                       \
        obs.obj_arel_long = 0;                                                                                           \
        obs.obj_arel_lat = 0;                                                                                            \
        obs.obj_class = 0;                                                                                               \
        obs.obj_orientation_angle = 0;                                                                                   \
        obs.obj_length = 0;                                                                                              \
        obs.obj_width = 0;                                                                                               \
        objects_[location].obstacle_list.push_back(obs);                                                                 \
    }                                                                                                                    \
} while(0)

//! @brief store object[index].status to local
#define CONTIRADAR_STORE_CLUSTER_0_STATUS(location)                                                                      \
case ID_Cluster_0_Status:                                                                                                \
{                                                                                                                        \
    /* cycle start judgements */                                                                                         \
    if(msg.id == ID_Cluster_0_Status)                                                                                    \
    {                                                                                                                    \
            /* start a new cycle */                                                                                      \
            if (location == 0)                                                                                           \
            {                                                                                                            \
                objects_[location].coord = CONTI_FRONT_CENTER;                                                           \
            }                                                                                                            \
            else if (location == 1)                                                                                      \
            {                                                                                                            \
                objects_[location].coord = CONTI_FRONT_LEFT;                                                             \
            }                                                                                                            \
            else if (location == 2)                                                                                      \
            {                                                                                                            \
                objects_[location].coord = CONTI_FRONT_RIGHT;                                                            \
            }                                                                                                            \
            else if (location == 3)                                                                                      \
            {                                                                                                            \
                objects_[location].coord = CONTI_REAR_LEFT;                                                              \
            }                                                                                                            \
            else if (location == 4)                                                                                      \
            {                                                                                                            \
                objects_[location].coord = CONTI_REAR_RIGHT;                                                             \
            }                                                                                                            \
            else if (location == 5)                                                                                      \
            {                                                                                                            \
                objects_[location].coord = CONTI_REAR_CENTER;                                                            \
            }                                                                                                            \
            objects_[location].timestamp.sec = msg.sec;                                                                  \
            objects_[location].timestamp.nsec = msg.nsec;                                                                \
            objects_[location].obstacle_list.clear();                                                                    \
                                                                                                                         \
            /* convert and push back object */                                                                           \
            if (number_of_objects_[location] != 0)                                                                       \
            {                                                                                                            \
                for (int index = 0; index < index_part1_[location] + 1; index++)                                         \
                {                                                                                                        \
                    CONTIRADAR_CONVERT_PUSH_CLUSTER(index, location);                                                    \
                }                                                                                                        \
                /* call service callback */                                                                              \
                /* end of this cycle */                                                                                  \
                if(ars048_obstacles_callback_[location])                                                                 \
                {                                                                                                        \
                    ars048_obstacles_callback_[location](objects_[location]);                                            \
                }                                                                                                        \
            }                                                                                                            \
    }                                                                                                                    \
    /* fetch object data */                                                                                              \
    for(int i = 0; i < index_part1_[location] + 1; i++)                                                                  \
    {                                                                                                                    \
        local_cluster_[location][i].part1_valid = false;                                                                 \
        local_cluster_[location][i].part2_valid = false;                                                                 \
    }                                                                                                                    \
    first_time_object_[location] = 1;                                                                                    \
    first_time_object_part1_[location] = 1;                                                                              \
    index_part1_[location] = 0;                                                                                          \
    index_part2_[location] = 0;                                                                                          \
    uint32_t near_num = GET_Cluster_0_Status_Cluster_NofClustersNear(msg.data);                                          \
    near_num = CALC_Cluster_0_Status_Cluster_NofClustersNear(near_num, 1);                                               \
    uint32_t far_num = GET_Cluster_0_Status_Cluster_NofClustersFar(msg.data);                                            \
    far_num = CALC_Cluster_0_Status_Cluster_NofClustersFar(far_num, 1);                                                  \
    number_of_objects_[location] = near_num + far_num;                                                                   \
    /*LOG(INFO) << "total num of clusters:" << number_of_objects_[location];*/                                           \
    break;                                                                                                               \
}

//! @brief store object[index].part1 to local
#define CONTIRADAR_STORE_CLUSTER_1_GENERAL(location)                                                                     \
case ID_Cluster_1_General:                                                                                               \
{                                                                                                                        \
    if(first_time_object_[location] == 1)                                                                                \
    {                                                                                                                    \
        if (first_time_object_part1_[location] == 1)                                                                     \
        {                                                                                                                \
            index_part1_[location] = 0;                                                                                  \
            first_time_object_part1_[location] = 0;                                                                      \
        }                                                                                                                \
        else                                                                                                             \
        {                                                                                                                \
            index_part1_[location] = index_part1_[location] + 1;                                                         \
        }                                                                                                                \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART1(index_part1_[location], 1, General, ID, msg.data, location);               \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART1(index_part1_[location], 1, General, DistLong, msg.data, location);         \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART1(index_part1_[location], 1, General, DistLat, msg.data, location);          \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART1(index_part1_[location], 1, General, VrelLong, msg.data, location);         \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART1(index_part1_[location], 1, General, VrelLat, msg.data, location);          \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART1(index_part1_[location], 1, General, DynProp, msg.data, location);          \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART1(index_part1_[location], 1, General, RCS, msg.data, location);              \
        local_cluster_[location][index_part1_[location]].part1_valid = true;                                             \
        break;                                                                                                           \
    }                                                                                                                    \
}

//! @brief store object[index].part2 to local
#define CONTIRADAR_STORE_CLUSTER_2_QUALITY(location)                                                                     \
case ID_Cluster_2_Quality:                                                                                               \
{                                                                                                                        \
    if (first_time_object_[location] == 1)                                                                               \
    {                                                                                                                    \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_ID(2, Quality, msg.data, location);                                              \
        for(int i = 0; i < index_part1_[location] + 1; i++)                                                              \
        {                                                                                                                \
            if(temp_cluster_[location].part2.ID == local_object_[location][i].part1.ID)                                  \
            {                                                                                                            \
                index_part2_[location] = i;                                                                              \
                local_cluster_[location][index_part2_[location]].part2.ID = temp_object_[location].part2.ID;             \
                break;                                                                                                   \
            }                                                                                                            \
        }                                                                                                                \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART2(index_part2_[location], 2, Quality, DistLong_rms, msg.data, location);     \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART2(index_part2_[location], 2, Quality, DistLat_rms, msg.data, location);      \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART2(index_part2_[location], 2, Quality, VrelLong_rms, msg.data, location);     \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART2(index_part2_[location], 2, Quality, VrelLat_rms, msg.data, location);      \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART2(index_part2_[location], 2, Quality, Pdh0, msg.data, location);             \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART2(index_part2_[location], 2, Quality, AmbigState, msg.data, location);       \
        CONTIRADAR_STORE_CLUSTER_SIGNAL_PART2(index_part2_[location], 2, Quality, InvalidState, msg.data, location);     \
        local_cluster_[location][index_part2_[location]].part2_valid = true;                                             \
        if (location == 0)                                                                                               \
        {                                                                                                                \
            local_cluster_[location][index_part2_[location]].coord = CONTI_FRONT_CENTER;                                 \
        }                                                                                                                \
        else if (location == 1)                                                                                          \
        {                                                                                                                \
            local_cluster_[location][index_part2_[location]].coord = CONTI_FRONT_LEFT;                                   \
        }                                                                                                                \
        else if (location == 2)                                                                                          \
        {                                                                                                                \
            local_cluster_[location][index_part2_[location]].coord = CONTI_FRONT_RIGHT;                                  \
        }                                                                                                                \
        else if (location == 3)                                                                                          \
        {                                                                                                                \
            local_cluster_[location][index_part2_[location]].coord = CONTI_REAR_LEFT;                                    \
        }                                                                                                                \
        else if (location == 4)                                                                                          \
        {                                                                                                                \
            local_cluster_[location][index_part2_[location]].coord = CONTI_REAR_RIGHT;                                   \
        }                                                                                                                \
        else if (location == 5)                                                                                          \
        {                                                                                                                \
            local_cluster_[location][index_part2_[location]].coord = CONTI_REAR_CENTER;                                  \
        }                                                                                                                \
        local_cluster_[location][index_part2_[location]].timestamp.sec = msg.sec;                                        \
        local_cluster_[location][index_part2_[location]].timestamp.nsec = msg.nsec;                                      \
        break;                                                                                                           \
    }                                                                                                                    \
}

//! @brief store and transmit object[index]
#define CONTIRADAR_STORE_CLUSTER(location)                                                                               \
    CONTIRADAR_STORE_CLUSTER_0_STATUS(location);                                                                         \
    CONTIRADAR_STORE_CLUSTER_1_GENERAL(location);                                                                        \
    CONTIRADAR_STORE_CLUSTER_2_QUALITY(location)                                                                                                                     

}   // namespace contiradar
}   // namespace sensors
}   // namespace holo

#endif
