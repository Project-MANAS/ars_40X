//
// Created by shivesh on 9/14/19.
//

#include "continental_radar/ros/radar_cfg_ros.hpp"

namespace continental_radar
{
RadarCfgROS::RadarCfgROS(ros::NodeHandle& nh, ContinentalRadarCAN * continental_radar_can) :
  continental_radar_can_(continental_radar_can)
{
  radar_cfg_ = continental_radar_can->get_radar_cfg();
  set_max_distance_service_ = nh.advertiseService("set_max_distance", &RadarCfgROS::set_max_distance, this);
  set_sensor_id_service_ = nh.advertiseService("set_sensor_id", &RadarCfgROS::set_sensor_id, this);
  set_radar_power_service_ = nh.advertiseService("set_radar_power", &RadarCfgROS::set_radar_power, this);
  set_output_type_service_ = nh.advertiseService("set_output_type", &RadarCfgROS::set_output_type, this);
  set_send_quality_service_ = nh.advertiseService("set_send_quality", &RadarCfgROS::set_send_quality, this);
  set_send_ext_info_service_ = nh.advertiseService("set_send_ext_info", &RadarCfgROS::set_send_ext_info, this);
  set_sort_index_service_ = nh.advertiseService("set_sort_index", &RadarCfgROS::set_sort_index, this);
  set_ctrl_relay_cfg_service_ = nh.advertiseService("set_ctrl_relay_cfg", &RadarCfgROS::set_ctrl_relay_cfg, this);
  set_store_in_nvm_service_ = nh.advertiseService("set_store_in_nvm", &RadarCfgROS::set_store_in_nvm, this);
  set_rcs_threshold_service_ = nh.advertiseService("set_rcs_threshold", &RadarCfgROS::set_rcs_threshold, this);
}

RadarCfgROS::~RadarCfgROS() {
}

bool RadarCfgROS::set_max_distance(
  MaxDistance::Request& req,
  MaxDistance::Response& /*res*/)
{
  radar_cfg_->set_max_distance(static_cast<uint64_t>(req.max_distance));
  continental_radar_can_->send_radar_data(can_messages::RadarCfg);
}

bool RadarCfgROS::set_sensor_id(
  SensorID::Request& req,
  SensorID::Response& /*res*/)
{
  radar_cfg_->set_sensor_id(req.sensor_id);
  continental_radar_can_->send_radar_data(can_messages::RadarCfg);
}

bool RadarCfgROS::set_radar_power(
  RadarPower::Request& req,
  RadarPower::Response& /*res*/)
{
  radar_cfg_->set_radar_power(static_cast<radar_cfg::RadarCfg_RadarPower>(req.radar_power));
  continental_radar_can_->send_radar_data(can_messages::RadarCfg);
}

bool RadarCfgROS::set_output_type(
  OutputType::Request& req,
  OutputType::Response& /*res*/)
{
  radar_cfg_->set_output_type(static_cast<radar_cfg::RadarCfg_OutputType>(req.output_type));
  continental_radar_can_->send_radar_data(can_messages::RadarCfg);
}

bool RadarCfgROS::set_send_quality(
  std_srvs::SetBool::Request& req,
  std_srvs::SetBool::Response& /*res*/)
{
  radar_cfg_->set_send_quality(static_cast<bool>(req.data));
  continental_radar_can_->send_radar_data(can_messages::RadarCfg);
}

bool RadarCfgROS::set_send_ext_info(
  std_srvs::SetBool::Request& req,
  std_srvs::SetBool::Response& /*res*/)
{
  radar_cfg_->set_send_ext_info(static_cast<bool>(req.data));
  continental_radar_can_->send_radar_data(can_messages::RadarCfg);
}

bool RadarCfgROS::set_sort_index(
  SortIndex::Request& req,
  SortIndex::Response& /*res*/)
{
  radar_cfg_->set_sort_index(static_cast<radar_cfg::RadarCfg_SortIndex>(req.sort_index));
  continental_radar_can_->send_radar_data(can_messages::RadarCfg);
}

bool RadarCfgROS::set_ctrl_relay_cfg(
  std_srvs::SetBool::Request& req,
  std_srvs::SetBool::Response& /*res*/)
{
  radar_cfg_->set_ctrl_relay_cfg(static_cast<bool>(req.data));
  continental_radar_can_->send_radar_data(can_messages::RadarCfg);
}

bool RadarCfgROS::set_store_in_nvm(
  std_srvs::SetBool::Request& req,
  std_srvs::SetBool::Response& /*res*/)
{
  radar_cfg_->set_store_in_nvm(static_cast<bool>(req.data));
  continental_radar_can_->send_radar_data(can_messages::RadarCfg);
}

bool RadarCfgROS::set_rcs_threshold(
  RCSThreshold::Request& req,
  RCSThreshold::Response& /*res*/)
{
  radar_cfg_->set_rcs_threshold(static_cast<radar_cfg::RadarCfg_RCS_Threshold>(req.rcs_threshold));
  continental_radar_can_->send_radar_data(can_messages::RadarCfg);
}

}
