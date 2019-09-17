//
// Created by shivesh on 9/14/19.
//

#include <continental_radar/motion_input_signals.hpp>

namespace continental_radar
{
SpeedInformation::SpeedInformation()
{
}

SpeedInformation::~SpeedInformation()
{
}

void SpeedInformation::set_speed(uint64_t speed)
{
  speed_information_msg.data.RadarDevice_Speed1 = speed >> 8;
  speed_information_msg.data.RadarDevice_Speed2 = speed & 0b11111111;
}

void SpeedInformation::set_speed_direction(RadarDevice_SpeedDirection direction)
{
  speed_information_msg.data.RadarDevice_SpeedDirection = direction;
}

YawRateInformation::YawRateInformation()
{
}

YawRateInformation::~YawRateInformation()
{
}

void YawRateInformation::set_yaw_rate(uint64_t yaw_rate)
{
  yaw_rate_information_msg.data.RadarDevice_YawRate1 = yaw_rate >> 8;
  yaw_rate_information_msg.data.RadarDevice_YawRate2 = yaw_rate & 0b11111111;
}
}
