/*! 
* \brief This file defines novatel class for message parsing and transferring
* \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
* \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice. */

#include <holo/sensors/novatel/novatel.h>
#include <glog/logging.h>

namespace holo
{
namespace sensors
{
namespace novatel
{

Novatel::Novatel(const std::string& com_type, const std::string& ip, const uint32_t& port,
        const std::string& dev, const uint32_t& baud_rate, const std::string& cmd)
{
    com_type_ = com_type;
    ip_ = ip;
    port_ = port;
    cmd_ = cmd;
    is_solution_good_ = false;
    memset(&b_msg_,0,SERIAL_BUFFER_SIZE);
    memset(&s_msg_,0,SERIAL_BUFFER_SIZE);
    msg_length_ = 0;

    if(com_type_ == std::string("tcp"))
    {
        if (port_ != ICOM1_PORT &&
            port_ != ICOM2_PORT &&
            port_ != ICOM3_PORT)
        {
            std::string error = "the input port is not correct, please chose one of these three ports: 3001, 3002, 3003";
            LOG(ERROR) << error;
           // throw HoloException(error);
        }

        if (!ConnectDevice())
        {

            std::string error = "could not connect to novatel device";
            LOG(ERROR) << error;

        }
        LOG(INFO)<<"connnect success";

        if (!SetCommand())
        {
            std::string error = "could not send commands to novatel device";
            LOG(ERROR) << error;

        }

        LOG(INFO)<<"setcommand success";

        read_thread_ptr_ = std::make_shared<std::thread >(std::bind(
                &Novatel::TcpReadMessages, this));

    }
    else if(com_type_ == std::string("serial"))
    {
        serial_dev_ = std::make_shared<serial::Serial>(
                dev,
                baud_rate,
                serial::Timeout(1, 10, 0, 1000, 0),
                serial::eightbits,
                serial::parity_none,
                serial::stopbits_one,
                serial::flowcontrol_none);

        read_thread_ptr_ = std::make_shared<std::thread >(std::bind(
                &Novatel::SerialReadMessages, this));
    }
    else
    {
       // HoloException("Novatel Invalid com_type!");
       std::cout<<"Novatel Invalid com_type!"<<std::endl;
    }


}

Novatel::~Novatel()
{
}



bool Novatel::ConnectDevice()
{
    if ((sockfd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        LOG(INFO) << "create socket error:" << strerror(errno) << "errno:" << errno;
        return false;
    }

    LOG(INFO) << "socket is created";
    memset(&servaddr_, 0, sizeof(servaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(port_);

    if (inet_pton(AF_INET, ip_.c_str(), &servaddr_.sin_addr) <= 0)
    {
        LOG(INFO) <<"ip:"<<  ip_.c_str()<<" port:"<<port_;
        LOG(ERROR) << "problem on address or port";
        return false;
    }

    LOG(INFO) << "address is imported";
    if (connect(sockfd_, (struct sockaddr*)&servaddr_, sizeof(servaddr_)) < 0)
    {
        LOG(ERROR) << "connect error:" << strerror(errno) << "(errno:" << errno;
        return false;
    }
    else
    {
        return true;
    }
}

bool Novatel::SetCommand()
{
    std::vector<std::string> commands;
    // commands are seperated with ";"
    boost::split(commands, cmd_, boost::is_any_of(";"), boost::token_compress_on);

    for (size_t i = 0; i != commands.size(); i++)
    {
        LOG(INFO)<<commands.at(i);
        commands.at(i) = commands.at(i) + "\r\n";
        std::vector<std::string> content;
        boost::split(content, commands.at(i), boost::is_any_of(" "), boost::token_compress_on);
        if(content.size() < 4 || content.at(0) != "log")
        {
            // the command is not command with frequency
            continue;
        }
        else if(content.at(1) == INSPVA_COMMAND_STR)
        {
           command_sample_time_[INSPVA_COMMAND_IDX] = std::atof(content.at(3).c_str());
        }
    }
    if (file_name_ != "")
    {
        std::string tmp_cmd;
        tmp_cmd = "logfile close";
        tmp_cmd = tmp_cmd + "\r\n";
        commands.push_back(tmp_cmd);

        tmp_cmd = "logfile open ";
        tmp_cmd = tmp_cmd + file_name_ + "\r\n";

        commands.push_back(tmp_cmd);
    }
    for (size_t i = 0; i < commands.size(); i++)
    {
        try
        {
            if(com_type_ == std::string("tcp"))
            {
                if (send(sockfd_, commands.at(i).c_str(), strlen(commands.at(i).c_str()),
                     0) < 0)
                {
                    LOG(ERROR) << "send msg error: " << strerror(errno) << "(errno:)" << errno;
                    continue;
                }
                else
                {
                    sleep(0.02);
                }
            }
            else
            {
                const char * cmd = commands.at(i).c_str();
                serial_dev_->write((uint8_t*)cmd, sizeof(cmd));
            }
            
        }
        catch (std::exception& e)
        {
            LOG(ERROR) << "Command sending error: " << e.what();
            return false;
        }
    }
    return true;
}

void Novatel::TcpReadMessages()
{
    size_t len;
    unsigned char buffer[MAX_SIZE];
    LOG(INFO)<<"start_readmessage";
    while (true)
    {
        try
        {
            len = recv(sockfd_, buffer, MAX_SIZE, 0);
            if(len > 0)
            {
               // LOG(INFO)<<"if in";
                 ParseMessages(buffer, len);
            }
            else
            {
                 LOG(ERROR) << "receive one message whose size is smaller than 0";
            }
        }
        catch (std::exception& e)
        {
            LOG(ERROR) << "ERROR reading port";
        }
    }
}



void Novatel::SerialReadMessages()
{
    uint32_t rbytes;
    unsigned char buffer[MAX_SIZE];
    while (true)
    {
        memset(buffer, 0, sizeof(buffer));
        while (!serial_dev_->waitReadable());
        rbytes = static_cast<uint32_t>(
            serial_dev_->read(buffer, sizeof(buffer)));
        if (rbytes > 0)
        {
            ParseMessages(buffer, rbytes);
        }
    }
}


void Novatel::ParseMessages(const unsigned char* msg,
                            const unsigned int& len)
{
    if (len > MAX_SIZE)
    {
        LOG(ERROR) << "the size of received message is too big";
        return;
    }
    //LOG(INFO)<<"msg[0]"<<msg[0]<<"msg[1]"<<msg[0];
     if(com_type_ == std::string("tcp"))
     {  
      // sleep(0.02);//改变频率


         if (msg[0] == SYNC_BYTE_1 &&
             msg[1] == SYNC_BYTE_2 &&
             (msg[2] == SYNC_BYTE_3 || msg[2] == SYNC_BYTE_3_SHORT)
                 ) {
         } else {
             // a message is received because the sync byte is incorrect
             LOG(INFO)<<" a message is received because the sync byte is incorrect";
             return;
         }

         BINARY_LOG_TYPE message_id = BINARY_LOG_TYPE(((*(msg + 5)) << 8) + *(msg + 4));

         // go into inspva mode
         static InsPositionVelocityAttitudeExtended raw_inspva;
         memcpy(&raw_inspva, msg, sizeof(InsPositionVelocityAttitudeExtended));
         if (!is_solution_good_) {
             //LOG(INFO)<<"raw_inspva.status"<,raw_inspva.status;
             if (raw_inspva.status == 3) {
                 is_solution_good_ = true;
                 LOG(INFO) << "The ins status is good";
             }
         } else {
             if (raw_inspva.status != 3) {
                 is_solution_good_ = false;
                 LOG(INFO) << "The ins status is not good";
             }
         }
         if (ins_pva_x_callback_) {
             ins_pva_x_callback_(raw_inspva);
         }
     }

     else if(com_type_ == std::string("serial"))
     {
          uint32_t sum = 0;
          uint32_t count = 0;
          memcpy(&b_msg_, &s_msg_, msg_length_);
          memset(&s_msg_, 0, SERIAL_BUFFER_SIZE);
          memcpy(&b_msg_[msg_length_], msg, len);
          sum = msg_length_ + len;
          while (1) {
                if (sum < INSPVAX_MSG_SIZE) {
                     msg_length_ = sum - count;
                     memcpy(&s_msg_, &b_msg_[count], msg_length_);
                     break;
                        }
                        if (b_msg_[0 + count] == SYNC_BYTE_1 &&
                            b_msg_[1 + count] == SYNC_BYTE_2 &&
                            b_msg_[2 + count] == SYNC_BYTE_3) {
                            uint16_t message_id = (b_msg_[5 + count] << 8) + b_msg_[4 + count];
                            if (message_id == INSPVAX_LOG_TYPE) {
                                static InsPositionVelocityAttitudeExtended raw_inspvax;
                                memcpy(&raw_inspvax, msg, sizeof(InsPositionVelocityAttitudeExtended));

                                if (!is_solution_good_) {
                                    if (raw_inspvax.ins_status == 3) {
                                        is_solution_good_ = true;
                                        LOG(INFO) << "The ins status is good";
                                    }
                                } else {
                                    if (raw_inspvax.ins_status != 3) {
                                        is_solution_good_ = false;
                                        LOG(INFO) << "The ins status is not good";
                                    }
                                }

                                if (ins_pva_x_callback_) {
                                    ins_pva_x_callback_(raw_inspvax);
                                }

                                count += INSPVAX_MSG_SIZE;
                                if (sum - count < INSPVAX_MSG_SIZE) {
                                    msg_length_ = sum - count;
                                    memcpy(&s_msg_, &b_msg_[count], msg_length_);
                                    break;
                                }
                            }
                        } else {
                            count++;
                        }
                    }
                    memset(&b_msg_, 0, SERIAL_BUFFER_SIZE);
             }
}

Timestamp Novatel::ConvertGpsToTimestamp(const Oem7Header& header)
{
    int64_t sec = std::floor(header.gps_millisecs/1e3);
    int64_t nsec = (header.gps_millisecs/1e3 - sec) * 1e9;
    sec = sec + header.gps_week * 7 * 24 * 3600;

    sec += nsec / 1000000000;
    nsec %= 1000000000;
    if(nsec < 0)
    {
        nsec += 1000000000;
        sec--;
    }
    if(sec < 0 || sec > std::numeric_limits<uint32_t>::max())
    {
        LOG(ERROR) << "Timestamp is out of 32-bit range";
    }

    Timestamp ts;
    ts.sec = uint32_t(sec);
    ts.nsec = uint32_t(nsec);    
    return ts;
}

}
}
}
