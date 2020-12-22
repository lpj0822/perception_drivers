/*!
* \brief This file defines delphi-esr class for objects parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef HOLO_SENSORS_DELPHI_ESR_H_
#define HOLO_SENSORS_DELPHI_ESR_H_

#include <stdio.h>
#include <unistd.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <holo/msg_header.h>
#include <holo/types.h>
#include <holo/yaml.h>
#include <holo/sensors/candev/candev.h>
#include <holo/sensors/delphi_esr/delphi_esr_structure.h>

namespace holo
{
namespace sensors
{
namespace delphi_esr
{

//! typedef
typedef std::function<void(const DelphiEsrObjectList&)> DelphiEsrCallback;

/**
 * @brief      defines a class for data analysing and transferring
 */
class DelphiEsr
{
public:
    //! driver constructor
    DelphiEsr(const std::string & config_yaml);

    //! @brief install delphi-esr objects service callback
    //! @param[in] callback the service routine of objects
    //! @param[in] converted the callback will installed to the converted
    //!            objects interface if set true
    //! @note usually called by an upper level application, e.g. ros node.
    void InstallServiceRawObsCallback(const DelphiEsrCallback& callback, uint8_t radar_index);

    //void DebugIfCanMessage(const holo::sensors::candev::CanMessage& msg);

private:
    //! the mounting location for delphi esr
    std::string radar_location_;
    //! local host server ip, like '192.168.1.3'
    std::string host_ip_;
    //! local host server port used, like '50030'
    std::string host_port_;
    //! remote client ip, like '192.168.1.106'
    std::string client_ip_;
    //! remote client port used, like '50030'
    std::string client_port_;

    //! local array for detected objects, used for store temporary
    //! objects information data
    DelphiEsrObjectList object_list_[ESR_NUM];

    //! can receiver smart pointer used for can data reception
    std::shared_ptr<holo::sensors::candev::CanReceiver> can_reciever_ptr_;

    //! objects callback function pointer, for objects data transferring
    DelphiEsrCallback raw_obstacles_callback_[ESR_NUM];

    //! @brief load parameters from yaml file
    //! @param[in] input full path string of input yaml file
    //! @return the result of loading parameters
    //! @retval true ok
    //! @retval false failed
    //! @note for the cpp-yaml, it crashed when target file or parameters
    //!       do not exist
    bool LoadParameters(const std::string input);

    /**
     * @brief      construct a detection from a can message
     *
     * @param[in]  msg        The message
     * @param      detection  The detection
     */
    void ConstructObject(const holo::sensors::candev::CanMessage& msg,
        DelphiEsrObject& object, uint8_t id);

    /**
     * @brief      Determines ability to message parsing.
     *
     * @param[in]  msg   The input can message
     * @note       this function usually installed to can_reciever_ptr_ through
     *             ::holo::sensors::candev::CanReceiver::InstallMessageCallback
     */
    void CanMessageParsing(const holo::sensors::candev::CanMessage& msg, enum DelphiEsrLocation location);

};  // class DelphiEsr

}   // namespace delphi_esr
}   // namespace sensors
}   // namespace holo

#endif
