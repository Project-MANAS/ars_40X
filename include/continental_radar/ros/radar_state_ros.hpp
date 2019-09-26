//
// Created by shivesh on 9/14/19.
//

#ifndef CONTINENTAL_RADAR_RADAR_STATE_ROS_HPP
#define CONTINENTAL_RADAR_RADAR_STATE_ROS_HPP

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <cstdint>

#include "continental_radar/continental_radar_can.hpp"
#include "continental_radar/RadarStatus.h"

namespace continental_radar
{
class RadarStateROS {
 public:
  RadarStateROS(ros::NodeHandle& nh, ContinentalRadarCAN * continental_radar_can);

  ~RadarStateROS();

  void send_radar_state();

 private:
  ros::Publisher radar_state_pub_;

  ContinentalRadarCAN * continental_radar_can_;

  radar_state::RadarState * radar_state_;
};
}

#endif //CONTINENTAL_RADAR_RADAR_STATE_ROS_HPP
