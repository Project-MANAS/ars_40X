//
// Created by shivesh on 9/14/19.
//

#include "ars_40X/ros/radar_state_ros.hpp"

namespace ars_40X {
RadarStateROS::RadarStateROS(ros::NodeHandle &nh, ARS_40X_CAN *ars_40X_can) :
    ars_40X_can_(ars_40X_can) {
  radar_state_ = ars_40X_can->get_radar_state();
  radar_state_pub_ = nh.advertise<RadarStatus>("radar_status", 1);
}

RadarStateROS::~RadarStateROS() {
}

void RadarStateROS::send_radar_state() {
  RadarStatus radar_status_msg;
  radar_status_msg.read_status = radar_state_->get_read_status();
  radar_status_msg.write_status = radar_state_->get_write_status();
  radar_status_msg.max_distance = radar_state_->get_max_distance();
  radar_status_msg.persistent_error = radar_state_->get_persistent_error_status();
  radar_status_msg.interference = radar_state_->get_interference_status();
  radar_status_msg.temperature_error = radar_state_->get_temperature_error_status();
  radar_status_msg.temporary_error = radar_state_->get_temporary_error_status();
  radar_status_msg.voltage_error = radar_state_->get_voltage_error_status();
  radar_status_msg.sensor_id = radar_state_->get_sensor_id();
  radar_status_msg.sort_index = radar_state_->get_sort_index();
  radar_status_msg.radar_power_cfg = radar_state_->get_radar_power_cfg();
  radar_status_msg.ctrl_relay_cfg = radar_state_->get_ctrl_relay_cfg();
  radar_status_msg.output_type_cfg = radar_state_->get_output_type_cfg();
  radar_status_msg.send_quality_cfg = radar_state_->get_send_quality_cfg();
  radar_status_msg.send_ext_info_cfg = radar_state_->get_ext_info_cfg();
  radar_status_msg.motion_rx_state = radar_state_->get_motion_rx_state();
  radar_status_msg.rcs_threshold = radar_state_->get_rcs_threshold();
  radar_state_pub_.publish(radar_status_msg);
}
}
