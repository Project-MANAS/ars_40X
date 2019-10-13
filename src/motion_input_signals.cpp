//
// Created by shivesh on 9/14/19.
//

#include <ars_40X/motion_input_signals.hpp>

namespace ars_40X {
namespace motion_input_signals {
SpeedInformation::SpeedInformation() {
}

SpeedInformation::~SpeedInformation() {
}

void SpeedInformation::set_speed(double speed) {
  int radar_speed = static_cast<int>(speed / 0.02);
  speed_information_msg.data.RadarDevice_Speed1 = static_cast<uint64_t>(radar_speed >> 8);
  speed_information_msg.data.RadarDevice_Speed2 = static_cast<uint64_t>(radar_speed & 255);
}

void SpeedInformation::set_speed_direction(RadarDevice_SpeedDirection direction) {
  speed_information_msg.data.RadarDevice_SpeedDirection = direction;
}

speed_information *SpeedInformation::get_speed_information() {
  return &speed_information_msg;
}

YawRateInformation::YawRateInformation() {
}

YawRateInformation::~YawRateInformation() {
}

void YawRateInformation::set_yaw_rate(double yaw_rate) {
  int radar_yaw_rate = static_cast<int>((yaw_rate + 327.68) / 0.01);
  yaw_rate_information_msg.data.RadarDevice_YawRate1 = static_cast<uint64_t>(radar_yaw_rate >> 8);
  yaw_rate_information_msg.data.RadarDevice_YawRate2 = static_cast<uint64_t>(radar_yaw_rate & 255);
}

yaw_rate_information *YawRateInformation::get_yaw_rate_information() {
  return &yaw_rate_information_msg;
}
}
}
