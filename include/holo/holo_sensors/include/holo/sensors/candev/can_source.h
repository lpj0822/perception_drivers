/*!
 *  \brief This file defines CanSource for can message source
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice */

#ifndef _HOLO_SENSORS_CAN_SOURCE_H_
#define _HOLO_SENSORS_CAN_SOURCE_H_

#include <type_traits>

namespace holo
{
namespace sensors
{
namespace candev
{

class CanSource
{
public:
    enum class Value : int
    {
        RESERVED = 0,   /* reserved/unknown */
        VEH_CH1  = 1,   /* vehicle can1 */
        VEH_CH2  = 2,   /* vehicle can2 */
        VEH_CH3 = 3,    /* vehicle can3 */
        VEH_CH4 = 4,    /* vehicle can4 */
        CAMERA  = 5,    /* camera */
        SRR_CF = 6,     /* srr center-front */
        SRR_FL = 7,     /* srr front-left */
        SRR_FR = 8,     /* srr front-right */
        SRR_RL = 9,     /* srr rear-left */
        SRR_RR = 10,    /* srr rear-right */
        SRR_CR = 11,    /* srr center-rear */
    };

    CanSource(): value_(Value::RESERVED) //constructor with default value
    {}

    CanSource(Value v): value_(v)
    {}

    CanSource(int i) //construct from underlying type
    {
        value_ = static_cast<Value>(i);
    }

    CanSource(std::string& s) //construct from human readable string
    {
        if (s == std::string("RESERVED"))
        {
            value_ = Value::RESERVED;
        }
        else if (s == std::string("VEH_CH1"))
        {
            value_ = Value::VEH_CH1;
        }
        else if (s == std::string("VEH_CH2"))
        {
            value_ = Value::VEH_CH2;
        }
        else if (s == std::string("VEH_CH3"))
        {
            value_ = Value::VEH_CH3;
        }
        else if (s == std::string("VEH_CH4"))
        {
            value_ = Value::VEH_CH4;
        }
        else if (s == std::string("CAMERA"))
        {
            value_ = Value::CAMERA;
        }
        else if (s == std::string("SRR_CR"))
        {
            value_ = Value::SRR_CR;
        }
        else if (s == std::string("SRR_FL"))
        {
            value_ = Value::SRR_FL;
        }
        else if (s == std::string("SRR_FR"))
        {
            value_ = Value::SRR_FR;
        }
        else if (s == std::string("SRR_RL"))
        {
            value_ = Value::SRR_RL;
        }
        else if (s == std::string("SRR_RR"))
        {
            value_ = Value::SRR_RR;
        }
        else if (s == std::string("SRR_CR"))
        {
            value_ = Value::SRR_CR;
        }
        else
        {
            value_ = Value::RESERVED;
        }
    }

    std::string String() const //cast to human readable string
    {
        switch (value_)
        {
        case Value::RESERVED:
            return std::string("RESERVED");
        case Value::VEH_CH1:
            return std::string("VEH_CH1");
        case Value::VEH_CH2:
            return std::string("VEH_CH2");
        case Value::VEH_CH3:
            return std::string("VEH_CH3");
        case Value::VEH_CH4:
            return std::string("VEH_CH4");
        case Value::CAMERA:
            return std::string("CAMERA");
        case Value::SRR_CF:
            return std::string("SRR_CF");
        case Value::SRR_FL:
            return std::string("SRR_FL");
        case Value::SRR_FR:
            return std::string("SRR_FR");
        case Value::SRR_RL:
            return std::string("SRR_RL");
        case Value::SRR_RR:
            return std::string("SRR_RR");
        case Value::SRR_CR:
            return std::string("SRR_CR");
        default:
            return std::string("RESERVED");
        }
    }

    std::underlying_type<Value>::type UnderLying() const //cast to underlying type
    {
        return static_cast<std::underlying_type<Value>::type>(value_);
    }

private:
    Value value_;
};

}
}
}
#endif
