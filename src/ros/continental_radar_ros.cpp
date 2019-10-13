//
// Created by shivesh on 9/13/19.
//

#include "continental_radar/ros/continental_radar_ros.hpp"

namespace continental_radar {
ContinentalRadarROS::ContinentalRadarROS(ros::NodeHandle& nh) :
  nh_(nh),
  cluster_list_ros_(nh_, this),
  motion_input_signals_ros_(nh_, this),
  object_list_ros_(nh_, this),
  radar_state_ros_(nh_, this),
  radar_cfg_ros_(nh_, this)
{
}

ContinentalRadarROS::~ContinentalRadarROS() {
}

void ContinentalRadarROS::receive_data() {
  while (ros::ok()) {
    receive_radar_data();
  }
}

void ContinentalRadarROS::send_cluster_0_status() {
  cluster_list_ros_.send_cluster_0_status();
}

void ContinentalRadarROS::send_cluster_1_general(){
  cluster_list_ros_.send_cluster_1_general();
}

void ContinentalRadarROS::send_object_0_status() {
  object_list_ros_.send_object_0_status();
}

void ContinentalRadarROS::send_object_1_general() {
  object_list_ros_.send_object_1_general();
}

void ContinentalRadarROS::send_object_2_quality() {
  object_list_ros_.send_object_2_quality();
}

void ContinentalRadarROS::send_object_3_extended() {
  object_list_ros_.send_object_3_extended();
}

void ContinentalRadarROS::send_radar_state() {
  radar_state_ros_.send_radar_state();
}

void ContinentalRadarROS::run() {
  receive_data_thread_ = std::thread(std::bind(&ContinentalRadarROS::receive_data, this));
  receive_data_thread_.detach();
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "continental_radar");
  ros::NodeHandle nh;
  continental_radar::ContinentalRadarROS continental_radar_ros_(nh);
  continental_radar_ros_.run();
  ros::spin();
}
