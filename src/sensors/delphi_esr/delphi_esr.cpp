/*!
* \brief This file defines  delphi-esr class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#include <glog/logging.h>
#include <holo/sensors/delphi_esr/delphi_esr.h>
#include <holo/sensors/delphi_esr/delphi_esr_dbc.h>

namespace holo
{
namespace sensors
{
namespace delphi_esr
{

DelphiEsr::DelphiEsr(const std::string & config_yaml)
{
    /* set callbacks as null */
    for(uint32_t i=0; i<ESR_NUM; i++)
    {
        raw_obstacles_callback_[i] = nullptr;
        memset(&object_list_[i], 0, sizeof(object_list_[i]));
    }

    LOG(INFO) << config_yaml;
    if(!LoadParameters(config_yaml))
    {
        LOG(FATAL) << "delphi esr driver failed to load config parameters!";
    }

    /* create can receiver */
    can_reciever_ptr_ = std::make_shared<holo::sensors::candev::CanReceiver>(
        "udp", host_ip_, host_port_, client_ip_, client_port_);

    if (radar_location_ == "front_center")
    {
        can_reciever_ptr_->InstallMessageCallback(
            std::bind(&DelphiEsr::CanMessageParsing, this, std::placeholders::_1, DelphiEsrLocation::FRONT_CENTER));        
    }
    else if (radar_location_ == "rear_center")
    {
        can_reciever_ptr_->InstallMessageCallback(
            std::bind(&DelphiEsr::CanMessageParsing, this, std::placeholders::_1, DelphiEsrLocation::REAR_CENTER));
    }
    else
    {
        LOG(FATAL) << "Failed to load delphi esr location!";
    }

}

bool DelphiEsr::LoadParameters(const std::string input)
{
    const std::string string_location = "RADAR_LOCATION";
    const std::string string_host_ip = "HOST_IP";
    const std::string string_host_port = "HOST_PORT";

    try
    {
        yaml::Node yaml_file = yaml::LoadFile(input);
        radar_location_      = yaml_file[string_location].as<std::string>();
        host_ip_             = yaml_file[string_host_ip].as<std::string>();
        host_port_           = yaml_file[string_host_port].as<std::string>();
    }
    catch(std::exception& e)
    {
        return false;
    }

    return true;
}

void DelphiEsr::ConstructObject(const holo::sensors::candev::CanMessage& msg,
    DelphiEsrObject& object, uint8_t id)
{
    /* fetch signals and sign detections */
    int16_t raw_u16val;
    int8_t raw_u8val;
    object.obj_id = id;
    object.timestamp.sec = msg.sec;
    object.timestamp.nsec = msg.nsec;
    raw_u16val = GET_ESR_Track01_CAN_TX_TRACK_MED_RANGE_MODE(msg.data);
    object.med_range_mode = CALC_ESR_Track01_CAN_TX_TRACK_MED_RANGE_MODE(raw_u16val, 1);
    raw_u16val = GET_ESR_Track01_CAN_TX_TRACK_STATUS(msg.data);
    object.status = CALC_ESR_Track01_CAN_TX_TRACK_STATUS(raw_u16val, 1);
    raw_u16val = GET_ESR_Track01_CAN_TX_TRACK_ANGLE(msg.data);
    object.angle = CALC_ESR_Track01_CAN_TX_TRACK_ANGLE(raw_u16val, 1.0);
    raw_u16val = GET_ESR_Track01_CAN_TX_TRACK_RANGE(msg.data);
    object.range = CALC_ESR_Track01_CAN_TX_TRACK_RANGE(raw_u16val, 1.0);
    raw_u16val = GET_ESR_Track01_CAN_TX_TRACK_RANGE_ACCEL(msg.data);
    object.range_accel = CALC_ESR_Track01_CAN_TX_TRACK_RANGE_ACCEL(raw_u16val, 1.0);
    raw_u16val = GET_ESR_Track01_CAN_TX_TRACK_RANGE_RATE(msg.data);
    object.range_rate = CALC_ESR_Track01_CAN_TX_TRACK_RANGE_RATE(raw_u16val, 1.0);
    raw_u16val = GET_ESR_Track01_CAN_TX_TRACK_WIDTH(msg.data);
    object.width = CALC_ESR_Track01_CAN_TX_TRACK_WIDTH(raw_u16val, 1.0);
    raw_u8val = GET_ESR_Track01_CAN_TX_TRACK_LAT_RATE(msg.data);
    object.lat_rate = CALC_ESR_Track01_CAN_TX_TRACK_LAT_RATE(raw_u8val, 1.0);
    raw_u16val = GET_ESR_Track01_CAN_TX_TRACK_BRIDGE_OBJECT(msg.data);
    object.is_bridge = CALC_ESR_Track01_CAN_TX_TRACK_BRIDGE_OBJECT(raw_u16val, 1);
    raw_u16val = GET_ESR_Track01_CAN_TX_TRACK_ONCOMING(msg.data);
    object.is_oncoming = CALC_ESR_Track01_CAN_TX_TRACK_ONCOMING(raw_u16val, 1);
    raw_u16val = GET_ESR_Track01_CAN_TX_TRACK_GROUPING_CHANGED(msg.data);
    object.is_grouping_changed = CALC_ESR_Track01_CAN_TX_TRACK_GROUPING_CHANGED(raw_u16val, 1);
}

void DelphiEsr::CanMessageParsing(const holo::sensors::candev::CanMessage& msg, enum DelphiEsrLocation location)
{
    switch(msg.id)
    { 
        case ID_ESR_Track01:
        {
            /* start a new cycle */
            object_list_[location].timestamp.sec = msg.sec;
            object_list_[location].timestamp.nsec = msg.nsec;
            if (location == FRONT_CENTER)
            {
                object_list_[location].coord_id = DELPHI_ESR_FRONT_CENTER;
            }
            else if (location == REAR_CENTER)
            {
                object_list_[location].coord_id = DELPHI_ESR_REAR_CENTER;
            }
            else
            {
                LOG(ERROR) << "undefined delphi esr coordinate!";
            }
            ConstructObject(msg, object_list_[location].objects[0], 0);
            break;
        }
        case ID_ESR_Track02:
            ConstructObject(msg, object_list_[location].objects[1], 1);
            break;
        case ID_ESR_Track03:
            ConstructObject(msg, object_list_[location].objects[2], 2);
            break;
        case ID_ESR_Track04:
            ConstructObject(msg, object_list_[location].objects[3], 3);
            break;
        case ID_ESR_Track05:
            ConstructObject(msg, object_list_[location].objects[4], 4);
            break;
        case ID_ESR_Track06:
            ConstructObject(msg, object_list_[location].objects[5], 5);
            break;
        case ID_ESR_Track07:
            ConstructObject(msg, object_list_[location].objects[6], 6);
            break;
        case ID_ESR_Track08:
            ConstructObject(msg, object_list_[location].objects[7], 7);
            break;  
        case ID_ESR_Track09:
            ConstructObject(msg, object_list_[location].objects[8], 8);
            break;  
        case ID_ESR_Track10:
            ConstructObject(msg, object_list_[location].objects[9], 9);
            break;
        case ID_ESR_Track11:
            ConstructObject(msg, object_list_[location].objects[10], 10);
            break;
        case ID_ESR_Track12:
            ConstructObject(msg, object_list_[location].objects[11], 11);
            break;
        case ID_ESR_Track13:
            ConstructObject(msg, object_list_[location].objects[12], 12);
            break;
        case ID_ESR_Track14:
            ConstructObject(msg, object_list_[location].objects[13], 13);
            break;
        case ID_ESR_Track15:
            ConstructObject(msg, object_list_[location].objects[14], 14);
            break;
        case ID_ESR_Track16:
            ConstructObject(msg, object_list_[location].objects[15], 15);
            break;
        case ID_ESR_Track17:
            ConstructObject(msg, object_list_[location].objects[16], 16);
            break;
        case ID_ESR_Track18:
            ConstructObject(msg, object_list_[location].objects[17], 17);
            break; 
        case ID_ESR_Track19:
            ConstructObject(msg, object_list_[location].objects[18], 18);
            break;
        case ID_ESR_Track20:
            ConstructObject(msg, object_list_[location].objects[19], 19);
            break;
        case ID_ESR_Track21:
            ConstructObject(msg, object_list_[location].objects[20], 20);
            break;                   
        case ID_ESR_Track22:
            ConstructObject(msg, object_list_[location].objects[21], 21);
            break;
        case ID_ESR_Track23:
            ConstructObject(msg, object_list_[location].objects[22], 22);
            break;
        case ID_ESR_Track24:
            ConstructObject(msg, object_list_[location].objects[23], 23);
            break;;
        case ID_ESR_Track25:
            ConstructObject(msg, object_list_[location].objects[24], 24);
            break;
        case ID_ESR_Track26:
            ConstructObject(msg, object_list_[location].objects[25], 25);
            break;
        case ID_ESR_Track27:
            ConstructObject(msg, object_list_[location].objects[26], 26);
            break;
        case ID_ESR_Track28:
            ConstructObject(msg, object_list_[location].objects[27], 27);
            break;
        case ID_ESR_Track29:
            ConstructObject(msg, object_list_[location].objects[28], 28);
            break;
        case ID_ESR_Track30:
            ConstructObject(msg, object_list_[location].objects[29], 29);
            break;
        case ID_ESR_Track31:
            ConstructObject(msg, object_list_[location].objects[30], 30);
            break;
        case ID_ESR_Track32:
            ConstructObject(msg, object_list_[location].objects[31], 31);
            break;
        case ID_ESR_Track33:
            ConstructObject(msg, object_list_[location].objects[32], 32);
            break;
        case ID_ESR_Track34:
            ConstructObject(msg, object_list_[location].objects[33], 33);
            break;
        case ID_ESR_Track35:
            ConstructObject(msg, object_list_[location].objects[34], 34);
            break;
        case ID_ESR_Track36:
            ConstructObject(msg, object_list_[location].objects[35], 35);
            break;
        case ID_ESR_Track37:
            ConstructObject(msg, object_list_[location].objects[36], 36);
            break;
        case ID_ESR_Track38:
            ConstructObject(msg, object_list_[location].objects[37], 37);
            break;
        case ID_ESR_Track39:
            ConstructObject(msg, object_list_[location].objects[38], 38);
            break;
        case ID_ESR_Track40:
            ConstructObject(msg, object_list_[location].objects[39], 39);
            break;
        case ID_ESR_Track41:
            ConstructObject(msg, object_list_[location].objects[40], 40);
            break;
        case ID_ESR_Track42:
            ConstructObject(msg, object_list_[location].objects[41], 41);
            break;
        case ID_ESR_Track43:
            ConstructObject(msg, object_list_[location].objects[42], 42);
            break;
        case ID_ESR_Track44:
            ConstructObject(msg, object_list_[location].objects[43], 43);
            break;
        case ID_ESR_Track45:
            ConstructObject(msg, object_list_[location].objects[44], 44);
            break;
        case ID_ESR_Track46:
            ConstructObject(msg, object_list_[location].objects[45], 45);
            break;
        case ID_ESR_Track47:
            ConstructObject(msg, object_list_[location].objects[46], 46);
            break;
        case ID_ESR_Track48:
            ConstructObject(msg, object_list_[location].objects[47], 47);
            break;
        case ID_ESR_Track49:
            ConstructObject(msg, object_list_[location].objects[48], 48);
            break;
        case ID_ESR_Track50:
            ConstructObject(msg, object_list_[location].objects[49], 49);
            break;
        case ID_ESR_Track51:
            ConstructObject(msg, object_list_[location].objects[50], 50);
            break;
        case ID_ESR_Track52:
            ConstructObject(msg, object_list_[location].objects[51], 51);
            break;
        case ID_ESR_Track53:
            ConstructObject(msg, object_list_[location].objects[52], 52);
            break;
        case ID_ESR_Track54:
            ConstructObject(msg, object_list_[location].objects[53], 53);
            break;
        case ID_ESR_Track55:
            ConstructObject(msg, object_list_[location].objects[54], 54);
            break;
        case ID_ESR_Track56:
            ConstructObject(msg, object_list_[location].objects[55], 55);
            break;
        case ID_ESR_Track57:
            ConstructObject(msg, object_list_[location].objects[56], 56);
            break;
        case ID_ESR_Track58:
            ConstructObject(msg, object_list_[location].objects[57], 57);
            break;
        case ID_ESR_Track59:
            ConstructObject(msg, object_list_[location].objects[58], 58);
            break;
        case ID_ESR_Track60:
            ConstructObject(msg, object_list_[location].objects[59], 59);
            break;
        case ID_ESR_Track61:
            ConstructObject(msg, object_list_[location].objects[60], 60);
            break;
        case ID_ESR_Track62:
            ConstructObject(msg, object_list_[location].objects[61], 61);
            break;
        case ID_ESR_Track63:
            ConstructObject(msg, object_list_[location].objects[62], 62);
            break;
        case ID_ESR_Track64:
        {
            ConstructObject(msg, object_list_[location].objects[63], 63);
            /* call service callback */
            if(raw_obstacles_callback_[location])
            {
               //LOG(INFO) << "L=" << raw_obstacles_[location].obstacle_list().size();
                raw_obstacles_callback_[location](object_list_[location]);
            }
            break;
        }
        default:
            break;
    }
}

void DelphiEsr::InstallServiceRawObsCallback(const DelphiEsrCallback& callback, uint8_t index)
{
    if(index > ESR_NUM - 1)
    {
        LOG(ERROR) << "install callback index" << uint32_t(index) << " out of range!";
        return;
    }

    raw_obstacles_callback_[index] = callback;

}

// void DelphiEsr::DebugIfCanMessage(const holo::sensors::candev::CanMessage& msg)
// {
//     CanMessageParsing(msg);
// }

}   // namespace bosch_mrr
}   // namespace sensors
}   // namespace holo
