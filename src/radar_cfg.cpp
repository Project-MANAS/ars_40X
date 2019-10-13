//
// Created by shivesh on 9/14/19.
//

#include <ars_40X/radar_cfg.hpp>

namespace ars_40X
{
namespace radar_cfg
{
RadarCfg::RadarCfg() {
  radar_cfg_msg.data.RadarCfg_MaxDistance_valid = 0;
  radar_cfg_msg.data.RadarCfg_SensorID_valid = 0;
  radar_cfg_msg.data.RadarCfg_RadarPower_valid = 0;
  radar_cfg_msg.data.RadarCfg_OutputType_valid = 0;
  radar_cfg_msg.data.RadarCfg_SendQuality_valid = 0;
  radar_cfg_msg.data.RadarCfg_SendExtInfo_valid = 0;
  radar_cfg_msg.data.RadarCfg_SortIndex_valid = 0;
  radar_cfg_msg.data.RadarCfg_CtrlRelay_valid = 0;
  radar_cfg_msg.data.RadarCfg_StoreInNVM_valid = 0;
  radar_cfg_msg.data.RadarCfg_RCS_Threshold_valid = 0;
}

RadarCfg::~RadarCfg() {
}

void RadarCfg::set_max_distance(uint64_t distance, bool valid) {
  distance = std::max(static_cast<uint64_t>(90), std::min(distance, static_cast<uint64_t>(1000)));
  distance /= 2;
  radar_cfg_msg.data.RadarCfg_MaxDistance1 = distance >> 2;
  radar_cfg_msg.data.RadarCfg_MaxDistance2 = distance & 0b11;
  radar_cfg_msg.data.RadarCfg_MaxDistance_valid = static_cast<uint64_t>(valid);
}

void RadarCfg::set_sensor_id(uint64_t id, bool valid) {
  radar_cfg_msg.data.RadarCfg_SensorID = std::max(static_cast<uint64_t>(0), std::min(id, static_cast<uint64_t>(7)));
  radar_cfg_msg.data.RadarCfg_SensorID_valid = static_cast<uint64_t>(valid);
}

void RadarCfg::set_radar_power(int power, bool valid) {
  radar_cfg_msg.data.RadarCfg_RadarPower = power;
  radar_cfg_msg.data.RadarCfg_RadarPower_valid = static_cast<uint64_t>(valid);
}

void RadarCfg::set_output_type(int output_type, bool valid) {
  radar_cfg_msg.data.RadarCfg_OutputType = output_type;
  radar_cfg_msg.data.RadarCfg_OutputType_valid = static_cast<uint64_t>(valid);
}

void RadarCfg::set_send_quality(bool quality, bool valid) {
  radar_cfg_msg.data.RadarCfg_SendQuality = static_cast<uint64_t>(quality);
  radar_cfg_msg.data.RadarCfg_SendQuality_valid = static_cast<uint64_t>(valid);
}

void RadarCfg::set_send_ext_info(bool send_ext, bool valid) {
  radar_cfg_msg.data.RadarCfg_SendExtInfo = static_cast<uint64_t>(send_ext);
  radar_cfg_msg.data.RadarCfg_SendExtInfo_valid = static_cast<uint64_t>(valid);
}

void RadarCfg::set_sort_index(int sort_index, bool valid) {
  radar_cfg_msg.data.RadarCfg_SortIndex = static_cast<uint64_t>(sort_index);
  radar_cfg_msg.data.RadarCfg_SortIndex_valid = static_cast<uint64_t>(valid);
}

void RadarCfg::set_ctrl_relay_cfg(bool ctrl_relay, bool valid) {
  radar_cfg_msg.data.RadarCfg_CtrlRelay = static_cast<uint64_t>(ctrl_relay);
  radar_cfg_msg.data.RadarCfg_CtrlRelay_valid = static_cast<uint64_t>(valid);
}

void RadarCfg::set_store_in_nvm(bool store_in_nvm, bool valid) {
  radar_cfg_msg.data.RadarCfg_StoreInNVM = static_cast<uint64_t>(store_in_nvm);
  radar_cfg_msg.data.RadarCfg_StoreInNVM_valid = static_cast<uint64_t>(valid);
}

void RadarCfg::set_rcs_threshold(int rcs_threshold, bool valid) {
  radar_cfg_msg.data.RadarCfg_RCS_Threshold = static_cast<uint64_t>(rcs_threshold);
  radar_cfg_msg.data.RadarCfg_RCS_Threshold_valid = static_cast<uint64_t>(valid);
}

radar_cfg *RadarCfg::get_radar_cfg() {
  return &radar_cfg_msg;
}
}
}
