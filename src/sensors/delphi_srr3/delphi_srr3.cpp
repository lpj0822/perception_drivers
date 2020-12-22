/*!
* \brief This file defines delphi-srr3 class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#include <glog/logging.h>
#include <holo/sensors/delphi_srr3/delphi_srr3.h>
#include <holo/sensors/delphi_srr3/delphi_srr3_dbc.h>

namespace holo
{
namespace sensors
{
namespace delphi_srr3
{

DelphiSrr3::DelphiSrr3(const std::string & config_yaml)
{
    /* set callbacks as null */
    for(uint32_t i=0; i<SRR3_SCANNERS_NUM; i++)
    {
        raw_obstacles_callback_[i] = nullptr;
        memset(&object_list_[i], 0, sizeof(object_list_[i]));
    }

    LOG(INFO) << config_yaml;
    if(!LoadParameters(config_yaml))
    {
        LOG(FATAL) << "delphi srr3 driver failed to load config parameters!";
    }

    /* create can receiver */
    can_reciever_ptr_ = std::make_shared<holo::sensors::candev::CanReceiver>(
        "udp", host_ip_, host_port_, client_ip_, client_port_);


    can_reciever_ptr_->InstallMessageCallback(
        std::bind(&DelphiSrr3::CanMessageParsing, this, std::placeholders::_1));        

}

bool DelphiSrr3::LoadParameters(const std::string input)
{
    const std::string string_location = "RADAR_LOCATION";
    const std::string string_host_ip = "HOST_IP";
    const std::string string_host_port = "HOST_PORT";

    try
    {
        yaml::Node yaml_file = yaml::LoadFile(input);
        host_ip_             = yaml_file[string_host_ip].as<std::string>();
        host_port_           = yaml_file[string_host_port].as<std::string>();
    }
    catch(std::exception& e)
    {
        return false;
    }

    return true;
}

void DelphiSrr3::ConstructObjectPartA(const holo::sensors::candev::CanMessage& msg,
    DelphiSrr3Object& object)
{
    /* fetch signals and sign detections */
    uint16_t raw_u16val;
    object.timestamp.sec = msg.sec;
    object.timestamp.nsec = msg.nsec;
    raw_u16val = GET_SRR_FL_392h_RdrObjAy_FL_1(msg.data);
    object.lateral_acceleration = CALC_SRR_FL_392h_RdrObjAy_FL_1(raw_u16val, 1.0);
    raw_u16val = GET_SRR_FL_392h_RdrObjAx_FL_1(msg.data);
    object.longitudinal_acceleration = CALC_SRR_FL_392h_RdrObjAx_FL_1(raw_u16val, 1.0);
    raw_u16val = GET_SRR_FL_392h_RdrObjCoast_FL_1(msg.data);
    object.is_coasted = CALC_SRR_FL_392h_RdrObjCoast_FL_1(raw_u16val, 1);
    raw_u16val = GET_SRR_FL_392h_RdrObjConf_FL_1(msg.data);
    object.confidence = CALC_SRR_FL_392h_RdrObjConf_FL_1(raw_u16val, 1);
    raw_u16val = GET_SRR_FL_392h_RdrObjMtnPat_FL_1(msg.data);
    object.object_motion_pattern = CALC_SRR_FL_392h_RdrObjMtnPat_FL_1(raw_u16val, 1);
    raw_u16val = GET_SRR_FL_392h_RdrObjID_FL_1(msg.data);
    object.obj_id = CALC_SRR_FL_392h_RdrObjID_FL_1(raw_u16val, 1);
    raw_u16val = GET_SRR_FL_392h_RdrObjTyp_FL_1(msg.data);
    object.object_type = CALC_SRR_FL_392h_RdrObjTyp_FL_1(raw_u16val, 1);
    raw_u16val = GET_SRR_FL_392h_RdrObjDy_FL_1(msg.data);
    object.lateral_position = CALC_SRR_FL_392h_RdrObjDy_FL_1(raw_u16val, 1.0);
}

void DelphiSrr3::ConstructObjectPartB(const holo::sensors::candev::CanMessage& msg,
    DelphiSrr3Object& object)
{
    /* fetch signals and sign detections */
    int16_t raw_u16val;
    raw_u16val = GET_SRR_FL_393h_RdrObjDx_FL_1(msg.data);
    object.longitudinal_position = CALC_SRR_FL_393h_RdrObjDx_FL_1(raw_u16val, 1.0);
    raw_u16val = GET_SRR_FL_393h_RdrObjTiAlv_FL_1(msg.data);
    object.alive_time = CALC_SRR_FL_393h_RdrObjTiAlv_FL_1(raw_u16val, 1);
    raw_u16val = GET_SRR_FL_393h_RdrObjVy_FL_1(msg.data);
    object.lateral_velocity = CALC_SRR_FL_393h_RdrObjVy_FL_1(raw_u16val, 1.0);
    raw_u16val = GET_SRR_FL_393h_RdrObjVx_FL_1(msg.data);
    object.longitudinal_velocity = CALC_SRR_FL_393h_RdrObjVx_FL_1(raw_u16val, 1.0);
    raw_u16val = GET_SRR_FL_393h_RdrObjWth_FL_1(msg.data);
    object.width = CALC_SRR_FL_393h_RdrObjWth_FL_1(raw_u16val, 1.0);
    raw_u16val = GET_SRR_FL_393h_RdrObjHeading_FL_1(msg.data);
    object.heading = CALC_SRR_FL_393h_RdrObjHeading_FL_1(raw_u16val, 1.0);
}

void DelphiSrr3::CanMessageParsing(const holo::sensors::candev::CanMessage& msg)
{
    switch(msg.id)
    { 
        /* SRR3 left scanner */
        case ID_SRR_FL_390h:
        {
            /* start a new cycle for front left srr3 */
            object_list_[SRR3_SCANNERS_LEFT].timestamp.sec = msg.sec;
            object_list_[SRR3_SCANNERS_LEFT].timestamp.nsec = msg.nsec;
            object_list_[SRR3_SCANNERS_LEFT].coord_id = DELPHI_SRR3_FRONT_LEFT;
            break;
        }
        case ID_SRR_RL_590h:
        {
            /* start a new cycle for rear left srr3 */
            object_list_[SRR3_SCANNERS_LEFT].timestamp.sec = msg.sec;
            object_list_[SRR3_SCANNERS_LEFT].timestamp.nsec = msg.nsec;
            object_list_[SRR3_SCANNERS_LEFT].coord_id = DELPHI_SRR3_REAR_LEFT;
            break;
        }
        case ID_SRR_FL_392h:
        case ID_SRR_RL_592h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[0]);
            break;
        case ID_SRR_FL_393h:
        case ID_SRR_RL_593h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[0]);
            break;    
        case ID_SRR_FL_394h:
        case ID_SRR_RL_594h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[1]);
            break;
        case ID_SRR_FL_395h:
        case ID_SRR_RL_595h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[1]);
            break;
        case ID_SRR_FL_396h:
        case ID_SRR_RL_596h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[2]);
            break;
        case ID_SRR_FL_397h:
        case ID_SRR_RL_597h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[2]);
            break;
        case ID_SRR_FL_398h:
        case ID_SRR_RL_598h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[3]);
            break;
        case ID_SRR_FL_399h:
        case ID_SRR_RL_599h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[3]);
            break;
        case ID_SRR_FL_39Ah:
        case ID_SRR_RL_59Ah:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[4]);
            break;
        case ID_SRR_FL_39Bh:
        case ID_SRR_RL_59Bh:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[4]);
            break;
        case ID_SRR_FL_39Ch:
        case ID_SRR_RL_59Ch:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[5]);
            break;
        case ID_SRR_FL_39Dh:
        case ID_SRR_RL_59Dh:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[5]);
            break;
        case ID_SRR_FL_39Eh:
        case ID_SRR_RL_59Eh:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[6]);
            break;
        case ID_SRR_FL_39Fh:
        case ID_SRR_RL_59Fh:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[6]);
            break;
        case ID_SRR_FL_3A0h:
        case ID_SRR_RL_5A0h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[7]);
            break;
        case ID_SRR_FL_3A1h:
        case ID_SRR_RL_5A1h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[7]);
            break;
        case ID_SRR_FL_3A2h:
        case ID_SRR_RL_5A2h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[8]);
            break;
        case ID_SRR_FL_3A3h:
        case ID_SRR_RL_5A3h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[8]);
            break;
        case ID_SRR_FL_3A4h:
        case ID_SRR_RL_5A4h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[9]);
            break;
        case ID_SRR_FL_3A5h:
        case ID_SRR_RL_5A5h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[9]);
            break;
        case ID_SRR_FL_3A6h:
        case ID_SRR_RL_5A6h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[10]);
            break;
        case ID_SRR_FL_3A7h:
        case ID_SRR_RL_5A7h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[10]);
            break;
        case ID_SRR_FL_3A8h:
        case ID_SRR_RL_5A8h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[11]);
            break;
        case ID_SRR_FL_3A9h:
        case ID_SRR_RL_5A9h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[11]);
            break;
        case ID_SRR_FL_3AAh:
        case ID_SRR_RL_5AAh:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[12]);
            break;
        case ID_SRR_FL_3ABh:
        case ID_SRR_RL_5ABh:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[12]);
            break;
        case ID_SRR_FL_3ACh:
        case ID_SRR_RL_5ACh:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[13]);
            break;
        case ID_SRR_FL_3ADh:
        case ID_SRR_RL_5ADh:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[13]);
            break;
        case ID_SRR_FL_3AEh:
        case ID_SRR_RL_5AEh:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_LEFT].objects[14]);
            break;
        case ID_SRR_FL_3AFh:
        case ID_SRR_RL_5AFh:
        {
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_LEFT].objects[14]);

            if(raw_obstacles_callback_[SRR3_SCANNERS_LEFT])
            {
                raw_obstacles_callback_[SRR3_SCANNERS_LEFT](object_list_[SRR3_SCANNERS_LEFT]);
            }
            break;
        }

        /* SRR3 right scanner */
        case ID_SRR_FR_3B0h:
        {
            /* start a new cycle for front right srr3 */
            object_list_[SRR3_SCANNERS_RIGHT].timestamp.sec = msg.sec;
            object_list_[SRR3_SCANNERS_RIGHT].timestamp.nsec = msg.nsec;
            object_list_[SRR3_SCANNERS_RIGHT].coord_id = DELPHI_SRR3_FRONT_RIGHT;
            break;
        }
        case ID_SRR_RR_5B0h:
        {
            /* start a new cycle for rear right srr3 */
            object_list_[SRR3_SCANNERS_RIGHT].timestamp.sec = msg.sec;
            object_list_[SRR3_SCANNERS_RIGHT].timestamp.nsec = msg.nsec;
            object_list_[SRR3_SCANNERS_RIGHT].coord_id = DELPHI_SRR3_REAR_RIGHT;
            break;
        }
        case ID_SRR_FR_3B2h:
        case ID_SRR_RR_5B2h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[0]);
            break;
        case ID_SRR_FR_3B3h:
        case ID_SRR_RR_5B3h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[0]);
            break;    
        case ID_SRR_FR_3B4h:
        case ID_SRR_RR_5B4h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[1]);
            break;
        case ID_SRR_FR_3B5h:
        case ID_SRR_RR_5B5h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[1]);
            break;
        case ID_SRR_FR_3B6h:
        case ID_SRR_RR_5B6h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[2]);
            break;
        case ID_SRR_FR_3B7h:
        case ID_SRR_RR_5B7h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[2]);
            break;
        case ID_SRR_FR_3B8h:
        case ID_SRR_RR_5B8h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[3]);
            break;
        case ID_SRR_FR_3B9h:
        case ID_SRR_RR_5B9h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[3]);
            break;
        case ID_SRR_FR_3BAh:
        case ID_SRR_RR_5BAh:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[4]);
            break;
        case ID_SRR_FR_3BBh:
        case ID_SRR_RR_5BBh:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[4]);
            break;
        case ID_SRR_FR_3BCh:
        case ID_SRR_RR_5BCh:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[5]);
            break;
        case ID_SRR_FR_3BDh:
        case ID_SRR_RR_5BDh:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[5]);
            break;
        case ID_SRR_FR_3BEh:
        case ID_SRR_RR_5BEh:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[6]);
            break;
        case ID_SRR_FR_3BFh:
        case ID_SRR_RR_5BFh:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[6]);
            break;
        case ID_SRR_FR_3C0h:
        case ID_SRR_RR_5C0h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[7]);
            break;
        case ID_SRR_FR_3C1h:
        case ID_SRR_RR_5C1h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[7]);
            break;
        case ID_SRR_FR_3C2h:
        case ID_SRR_RR_5C2h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[8]);
            break;
        case ID_SRR_FR_3C3h:
        case ID_SRR_RR_5C3h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[8]);
            break;
        case ID_SRR_FR_3C4h:
        case ID_SRR_RR_5C4h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[9]);
            break;
        case ID_SRR_FR_3C5h:
        case ID_SRR_RR_5C5h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[9]);
            break;
        case ID_SRR_FR_3C6h:
        case ID_SRR_RR_5C6h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[10]);
            break;
        case ID_SRR_FR_3C7h:
        case ID_SRR_RR_5C7h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[10]);
            break;
        case ID_SRR_FR_3C8h:
        case ID_SRR_RR_5C8h:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[11]);
            break;
        case ID_SRR_FR_3C9h:
        case ID_SRR_RR_5C9h:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[11]);
            break;
        case ID_SRR_FR_3CAh:
        case ID_SRR_RR_5CAh:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[12]);
            break;
        case ID_SRR_FR_3CBh:
        case ID_SRR_RR_5CBh:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[12]);
            break;
        case ID_SRR_FR_3CCh:
        case ID_SRR_RR_5CCh:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[13]);
            break;
        case ID_SRR_FR_3CDh:
        case ID_SRR_RR_5CDh:
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[13]);
            break;
        case ID_SRR_FR_3CEh:
        case ID_SRR_RR_5CEh:
            ConstructObjectPartA(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[14]);
            break;
        case ID_SRR_FR_3CFh:
        case ID_SRR_RR_5CFh:
        {
            ConstructObjectPartB(msg, object_list_[SRR3_SCANNERS_RIGHT].objects[14]);

            if(raw_obstacles_callback_[SRR3_SCANNERS_RIGHT])
            {
                raw_obstacles_callback_[SRR3_SCANNERS_RIGHT](object_list_[SRR3_SCANNERS_RIGHT]);
            }
            break;
        }
        default:
            break;
    }
}

void DelphiSrr3::InstallServiceRawObsCallback(const DelphiSrr3Callback& callback, uint8_t index)
{
    if(index > SRR3_SCANNERS_NUM - 1)
    {
        LOG(ERROR) << "install callback index" << uint32_t(index) << " out of range!";
        return;
    }

    raw_obstacles_callback_[index] = callback;

}

// void DelphiSrr3::DebugIfCanMessage(const holo::sensors::candev::CanMessage& msg)
// {
//     CanMessageParsing(msg);
// }

}   // namespace delphi_srr3
}   // namespace sensors
}   // namespace holo
