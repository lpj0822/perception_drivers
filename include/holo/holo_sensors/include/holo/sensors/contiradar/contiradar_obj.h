/*!
* \brief This file defines Contiradar class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLO_SENSORS_CONTIRADAR_OBJ_H_
#define HOLO_SENSORS_CONTIRADAR_OBJ_H_

namespace holo
{
namespace sensors
{
namespace contiradar
{

//! @brief fetch object signal of object[index].partno.signal from 'data'
//!        and store to local
#define CONTIRADAR_STORE_OBJECT_SIGNAL_ID(partno, type, data, location)   do {                                           \
temp_object_[location].part##partno.ID =                                                                                 \
GET_Object_##partno##_##type##_Object_ID(data);                                                                          \
} while(0)

#define CONTIRADAR_STORE_OBJECT_SIGNAL_PART1(index1, partno, type, signal, data, location)   do {                        \
local_object_[location][index1].part##partno.signal =                                                                    \
GET_Object_##partno##_##type##_Object_##signal(data);                                                                    \
} while(0)                                                              

#define CONTIRADAR_STORE_OBJECT_SIGNAL_PART2(index1, partno, type, signal, data, location)   do {                        \
local_object_[location][index1].part##partno.signal =                                                                    \
GET_Object_##partno##_##type##_Object_##signal(data);                                                                    \
} while(0)

#define CONTIRADAR_STORE_OBJECT_SIGNAL_PART3(index1, partno, type, signal, data, location)   do {                        \
local_object_[location][index1].part##partno.signal =                                                                    \
GET_Object_##partno##_##type##_Object_##signal(data);                                                                    \
} while(0)

//! @brief calculate physical value of object[index].partno.signal
#define CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(index1, partno, type, signal, location)                                        \
CALC_Object_##partno##_##type##_Object_##signal(local_object_[location][index1].part##partno.signal, 1.0)                          

//! @brief fetch local data and convert these data to standard format one,
//!        and push them into array for transmission
#define CONTIRADAR_CONVERT_PUSH_OBJECT(index1, location)    do {                                                         \
    /* check if all of part1~3 are received */                                                                           \
    Ars048Obstacle obs;                                                                                                  \
    /*if(local_object_[location][index1].part1_valid == 1 && local_object_[location][index1].part2_valid == 1 &&       */\
    /*   local_object_[location][index1].part3_valid == 1)                                                             */\
    {                                                                                                                    \
        obs.timestamp = local_object_[location][index].timestamp;                                                        \
        obs.coord = local_object_[location][index].coord;                                                                \
        obs.obj_id  = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                                 \
            index, 1, General, ID, location);                                                                            \
        obs.obj_dist_long = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                           \
            index, 1, General, DistLong, location);                                                                      \
        obs.obj_dist_lat = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                            \
            index, 1, General, DistLat, location);                                                                       \
        obs.obj_vrel_long = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                           \
            index, 1, General, VrelLong, location);                                                                      \
        obs.obj_vrel_lat = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                            \
            index, 1, General, VrelLat, location);                                                                       \
        obs.obj_dyn_prop = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                            \
            index, 1, General, DynProp, location);                                                                       \
        obs.obj_rcs = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                                 \
            index, 1, General, RCS, location);                                                                           \
        obs.obj_dist_long_rms = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                       \
            index, 2, Quality, DistLong_rms, location);                                                                  \
        obs.obj_dist_lat_rms = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                        \
            index, 2, Quality, DistLat_rms, location);                                                                   \
        obs.obj_vrel_long_rms = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                       \
            index, 2, Quality, VrelLong_rms, location);                                                                  \
        obs.obj_vrel_lat_rms = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                        \
            index, 2, Quality, VrelLat_rms, location);                                                                   \
        obs.obj_arel_long_rms = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                       \
            index, 2, Quality, ArelLong_rms, location);                                                                  \
        obs.obj_arel_lat_rms = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                        \
            index, 2, Quality, ArelLat_rms, location);                                                                   \
        obs.obj_orientation_rms = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                     \
            index, 2, Quality, Orientation_rms, location);                                                               \
        obs.obj_meas_state = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                          \
            index, 2, Quality, MeasState, location);                                                                     \
        obs.obj_prob_of_exist = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                       \
            index, 2, Quality, ProbOfexist, location);                                                                   \
        obs.obj_arel_long = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                           \
            index, 3, Extended, ArelLong, location);                                                                     \
        obs.obj_arel_lat = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                            \
            index, 3, Extended, ArelLat, location);                                                                      \
        obs.obj_class = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                               \
            index, 3, Extended, Class, location);                                                                        \
        obs.obj_orientation_angle = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                   \
            index, 3, Extended, OrientationAngle, location);                                                             \
        obs.obj_length = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                              \
            index, 3, Extended, Length, location);                                                                       \
        obs.obj_width = CONTIRADAR_LOAD_OBJECT_SIGNAL_PHY(                                                               \
            index, 3, Extended, Width, location);                                                                        \
        obs.pdh0 = 0;                                                                                                    \
        obs.ambig_state = 0;                                                                                             \
        obs.invalid_state = 0;                                                                                           \
                                                                                                                         \
        objects_[location].obstacle_list.push_back(obs);                                                                 \
    }                                                                                                                    \
} while(0)

//! @brief store object[index].status to local
#define CONTIRADAR_STORE_OBJECT_0_STATUS(location)                                                                       \
case ID_Object_0_Status:                                                                                                 \
{                                                                                                                        \
    /* cycle start judgements */                                                                                         \
    if(msg.id == ID_Object_0_Status)                                                                                     \
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
                    CONTIRADAR_CONVERT_PUSH_OBJECT(index, location);                                                     \
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
        local_object_[location][i].part1_valid = false;                                                                  \
        local_object_[location][i].part2_valid = false;                                                                  \
        local_object_[location][i].part3_valid = false;                                                                  \
    }                                                                                                                    \
    first_time_object_[location] = 1;                                                                                    \
    first_time_object_part1_[location] = 1;                                                                              \
    index_part1_[location] = 0;                                                                                          \
    index_part2_[location] = 0;                                                                                          \
    index_part3_[location] = 0;                                                                                          \
    number_of_objects_[location] = GET_Object_0_Status_Object_NofObjects(msg.data);                                      \
    break;                                                                                                               \
}

//! @brief store object[index].part1 to local
#define CONTIRADAR_STORE_OBJECT_1_GENERAL(location)                                                                      \
case ID_Object_1_General:                                                                                                \
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
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART1(index_part1_[location], 1, General, ID, msg.data, location);                \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART1(index_part1_[location], 1, General, DistLong, msg.data, location);          \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART1(index_part1_[location], 1, General, DistLat, msg.data, location);           \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART1(index_part1_[location], 1, General, VrelLong, msg.data, location);          \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART1(index_part1_[location], 1, General, VrelLat, msg.data, location);           \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART1(index_part1_[location], 1, General, DynProp, msg.data, location);           \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART1(index_part1_[location], 1, General, RCS, msg.data, location);               \
        local_object_[location][index_part1_[location]].part1_valid = true;                                              \
        break;                                                                                                           \
    }                                                                                                                    \
}

//! @brief store object[index].part2 to local
#define CONTIRADAR_STORE_OBJECT_2_QUALITY(location)                                                                      \
case ID_Object_2_Quality:                                                                                                \
{                                                                                                                        \
    if (first_time_object_[location] == 1)                                                                               \
    {                                                                                                                    \
        CONTIRADAR_STORE_OBJECT_SIGNAL_ID(2, Quality, msg.data, location);                                               \
        for(int i = 0; i < index_part1_[location] + 1; i++)                                                              \
        {                                                                                                                \
            if(temp_object_[location].part2.ID == local_object_[location][i].part1.ID)                                   \
            {                                                                                                            \
                index_part2_[location] = i;                                                                              \
                local_object_[location][index_part2_[location]].part2.ID = temp_object_[location].part2.ID;              \
                break;                                                                                                   \
            }                                                                                                            \
        }                                                                                                                \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART2(index_part2_[location], 2, Quality, DistLong_rms, msg.data, location);      \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART2(index_part2_[location], 2, Quality, DistLat_rms, msg.data, location);       \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART2(index_part2_[location], 2, Quality, VrelLong_rms, msg.data, location);      \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART2(index_part2_[location], 2, Quality, VrelLat_rms, msg.data, location);       \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART2(index_part2_[location], 2, Quality, ArelLong_rms, msg.data, location);      \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART2(index_part2_[location], 2, Quality, ArelLat_rms, msg.data, location);       \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART2(index_part2_[location], 2, Quality, Orientation_rms, msg.data, location);   \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART2(index_part2_[location], 2, Quality, MeasState, msg.data, location);         \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART2(index_part2_[location], 2, Quality, ProbOfexist, msg.data, location);       \
        local_object_[location][index_part2_[location]].part2_valid = true;                                              \
        break;                                                                                                           \
    }                                                                                                                    \
}

//! @brief store object[index].part3 to local
#define CONTIRADAR_STORE_OBJECT_3_EXTENDED(location)                                                                     \
case ID_Object_3_Extended:                                                                                               \
{                                                                                                                        \
    if (first_time_object_[location] == 1)                                                                               \
    {                                                                                                                    \
        CONTIRADAR_STORE_OBJECT_SIGNAL_ID(3, Extended, msg.data, location);                                              \
        for(int i = 0; i < index_part1_[location] + 1; i++)                                                              \
        {                                                                                                                \
            if(temp_object_[location].part3.ID == local_object_[location][i].part1.ID)                                   \
            {                                                                                                            \
                index_part3_[location] = i;                                                                              \
                local_object_[location][index_part3_[location]].part3.ID = temp_object_[location].part3.ID;              \
                break;                                                                                                   \
            }                                                                                                            \
        }                                                                                                                \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART3(index_part3_[location], 3, Extended, ArelLong, msg.data, location);         \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART3(index_part3_[location], 3, Extended, ArelLat, msg.data, location);          \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART3(index_part3_[location], 3, Extended, Class, msg.data, location);            \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART3(index_part3_[location], 3, Extended, OrientationAngle, msg.data, location); \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART3(index_part3_[location], 3, Extended, Length, msg.data, location);           \
        CONTIRADAR_STORE_OBJECT_SIGNAL_PART3(index_part3_[location], 3, Extended, Width, msg.data, location);            \
        local_object_[location][index_part3_[location]].part3_valid = true;                                              \
        if (location == 0)                                                                                               \
        {                                                                                                                \
            local_object_[location][index_part3_[location]].coord = CONTI_FRONT_CENTER;                                  \
        }                                                                                                                \
        else if (location == 1)                                                                                          \
        {                                                                                                                \
            local_object_[location][index_part3_[location]].coord = CONTI_FRONT_LEFT;                                    \
        }                                                                                                                \
        else if (location == 2)                                                                                          \
        {                                                                                                                \
            local_object_[location][index_part3_[location]].coord = CONTI_FRONT_RIGHT;                                   \
        }                                                                                                                \
        else if (location == 3)                                                                                          \
        {                                                                                                                \
            local_object_[location][index_part3_[location]].coord = CONTI_REAR_LEFT;                                     \
        }                                                                                                                \
        else if (location == 4)                                                                                          \
        {                                                                                                                \
            local_object_[location][index_part3_[location]].coord = CONTI_REAR_RIGHT;                                    \
        }                                                                                                                \
        else if (location == 5)                                                                                          \
        {                                                                                                                \
            local_object_[location][index_part3_[location]].coord = CONTI_REAR_CENTER;                                   \
        }                                                                                                                \
        local_object_[location][index_part3_[location]].timestamp.sec = msg.sec;                                         \
        local_object_[location][index_part3_[location]].timestamp.nsec = msg.nsec;                                       \
        break;                                                                                                           \
    }                                                                                                                    \
}

//! @brief store and transmit object[index]
#define CONTIRADAR_STORE_OBJECT(location)                                                                                \
    CONTIRADAR_STORE_OBJECT_0_STATUS(location);                                                                          \
    CONTIRADAR_STORE_OBJECT_1_GENERAL(location);                                                                         \
    CONTIRADAR_STORE_OBJECT_2_QUALITY(location);                                                                         \
    CONTIRADAR_STORE_OBJECT_3_EXTENDED(location)                                             

}   // namespace contiradar
}   // namespace sensors
}   // namespace holo

#endif
