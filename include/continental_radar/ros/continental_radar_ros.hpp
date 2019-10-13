//
// Created by shivesh on 9/13/19.
//

#ifndef CONTINENTAL_RADAR_CONTINENTAL_RADAR_ROS_HPP
#define CONTINENTAL_RADAR_CONTINENTAL_RADAR_ROS_HPP

#include <ros/ros.h>
#include <thread>

#include "cluster_list_ros.hpp"
#include "motion_input_signals_ros.hpp"
#include "object_list_ros.hpp"
#include "radar_cfg_ros.hpp"
#include "radar_state_ros.hpp"
#include "continental_radar/continental_radar_can.hpp"

namespace ars_40X
{
class ContinentalRadarROS : public ContinentalRadarCAN {
 public:
  ContinentalRadarROS(ros::NodeHandle& nh);

  ~ContinentalRadarROS();

  void receive_data();

  void run();

  void send_cluster_0_status();

  void send_cluster_1_general();

  void send_object_0_status();

  void send_object_1_general();

  void send_object_2_quality();

  void send_object_3_extended();

  void send_radar_state();

 private:
  ros::NodeHandle nh_;

  std::thread receive_data_thread_;

  ClusterListROS cluster_list_ros_;

  MotionInputSignalsROS motion_input_signals_ros_;

  ObjectListROS object_list_ros_;

  RadarCfgROS radar_cfg_ros_;

  RadarStateROS radar_state_ros_;
};
}

#endif //CONTINENTAL_RADAR_CONTINENTAL_RADAR_ROS_HPP
