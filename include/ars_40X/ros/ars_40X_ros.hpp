//
// Created by shivesh on 9/13/19.
//

#ifndef ARS_40X_ARS_40X_ROS_HPP
#define ARS_40X_ARS_40X_ROS_HPP

#include <ros/ros.h>
#include <thread>

#include "ars_40X/ros/cluster_list_ros.hpp"
#include "ars_40X/ros/motion_input_signals_ros.hpp"
#include "ars_40X/ros/object_list_ros.hpp"
#include "ars_40X/ros/radar_cfg_ros.hpp"
#include "ars_40X/ros/radar_state_ros.hpp"

#include "ars_40X/ars_40X_can.hpp"

namespace ars_40X {
class ARS_40X_ROS : public ARS_40X_CAN {
 public:
  ARS_40X_ROS(ros::NodeHandle &nh);

  ~ARS_40X_ROS();

  void receive_data();

  void run();

  void send_cluster_0_status();

  void send_cluster_1_general();

  void send_cluster_2_quality();

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

#endif //ARS_40X_ARS_40X_ROS_HPP
