//
// Created by shivesh on 9/14/19.
//

#ifndef CONTINENTAL_RADAR_MOTION_INPUT_SIGNALS_ROS_HPP
#define CONTINENTAL_RADAR_MOTION_INPUT_SIGNALS_ROS_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <cstdint>

#include "continental_radar/continental_radar_can.hpp"

namespace continental_radar
{
class MotionInputSignalsROS
{
 public:
  MotionInputSignalsROS(ros::NodeHandle& nh, ContinentalRadarCAN * continental_radar_can);

  ~MotionInputSignalsROS();

 private:
  void odom_callback(nav_msgs::Odometry msg);

  ContinentalRadarCAN * continental_radar_can_;

  motion_input_signals::SpeedInformation * speed_information_;

  motion_input_signals::YawRateInformation * yaw_rate_information_;

  ros::Subscriber odom_sub_;

  double yaw_vel_prev_;

  double yaw_vel_time_prev_;
};
}

#endif //CONTINENTAL_RADAR_MOTION_INPUT_SIGNALS_ROS_HPP
