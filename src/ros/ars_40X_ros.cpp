//
// Created by shivesh on 9/13/19.
//

#include "ars_40X/ros/ars_40X_ros.hpp"

namespace ars_40X {
ARS_40X_ROS::ARS_40X_ROS(ros::NodeHandle &nh) :
    nh_(nh),
    cluster_list_ros_(nh_, this),
    motion_input_signals_ros_(nh_, this),
    object_list_ros_(nh_, this),
    radar_cfg_ros_(nh_, this),
    radar_state_ros_(nh_, this) {
  ros::NodeHandle private_nh("~");
  std::string frame_id;
  private_nh.param<std::string>("frame_id", frame_id, std::string("radar"));
  cluster_list_ros_.set_frame_id(frame_id);
  object_list_ros_.set_frame_id(frame_id);
}

ARS_40X_ROS::~ARS_40X_ROS() {
}

void ARS_40X_ROS::receive_data() {
  while (ros::ok()) {
    receive_radar_data();
  }
}

void ARS_40X_ROS::send_cluster_0_status() {
  cluster_list_ros_.send_cluster_0_status();
}

void ARS_40X_ROS::send_cluster_1_general() {
  cluster_list_ros_.send_cluster_1_general();
}

void ARS_40X_ROS::send_cluster_2_quality() {
  cluster_list_ros_.send_cluster_2_quality();
}

void ARS_40X_ROS::send_object_0_status() {
  object_list_ros_.send_object_0_status();
}

void ARS_40X_ROS::send_object_1_general() {
  object_list_ros_.send_object_1_general();
}

void ARS_40X_ROS::send_object_2_quality() {
  object_list_ros_.send_object_2_quality();
}

void ARS_40X_ROS::send_object_3_extended() {
  object_list_ros_.send_object_3_extended();
}

void ARS_40X_ROS::send_radar_state() {
  radar_state_ros_.send_radar_state();
}

void ARS_40X_ROS::run() {
  receive_data_thread_ = std::thread(std::bind(&ARS_40X_ROS::receive_data, this));
  receive_data_thread_.detach();
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ars_40X_ros");
  ros::NodeHandle nh;
  ars_40X::ARS_40X_ROS ars_40X_ros_(nh);
  ars_40X_ros_.run();
  ros::spin();
}
