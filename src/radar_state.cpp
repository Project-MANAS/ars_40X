//
// Created by shivesh on 9/14/19.
//

#include <ars_40X/radar_state.hpp>

#include <iostream>

namespace ars_40X {
namespace radar_state {
RadarState::RadarState() {
}

RadarState::~RadarState() {
}

bool RadarState::get_read_status() {
  return static_cast<bool>(radar_state_msg.data.RadarState_NVMReadStatus);
}

bool RadarState::get_write_status() {
  return static_cast<bool>(radar_state_msg.data.RadarState_NVMwriteStatus);
}

uint64_t RadarState::get_max_distance() {
  return (radar_state_msg.data.RadarState_MaxDistanceCfg1 << 2 |
      radar_state_msg.data.RadarState_MaxDistanceCfg2) * 2;
}

bool RadarState::get_persistent_error_status() {
  return static_cast<bool>(radar_state_msg.data.RadarState_Persistent_Error);
}

bool RadarState::get_interference_status() {
  return static_cast<bool>(radar_state_msg.data.RadarState_Interference);
}

bool RadarState::get_temperature_error_status() {
  return static_cast<bool>(radar_state_msg.data.RadarState_Temperature_Error);
}

bool RadarState::get_temporary_error_status() {
  return static_cast<bool>(radar_state_msg.data.RadarState_Temporary_Error);
}

bool RadarState::get_voltage_error_status() {
  return static_cast<bool>(radar_state_msg.data.RadarState_Voltage_Error);
}

int RadarState::get_sensor_id() {
  return static_cast<int>(radar_state_msg.data.RadarState_SensorID);
}

int RadarState::get_sort_index() {
  return static_cast<int>(radar_state_msg.data.RadarState_SortIndex);
}

int RadarState::get_radar_power_cfg() {
  return static_cast<int>((radar_state_msg.data.RadarState_RadarPowerCfg1 << 1)
        | radar_state_msg.data.RadarState_RadarPowerCfg2);
}

bool RadarState::get_ctrl_relay_cfg() {
  return static_cast<bool>(radar_state_msg.data.RadarState_CtrlRelayCfg);
}

int RadarState::get_output_type_cfg() {
  return static_cast<int>(radar_state_msg.data.RadarState_OutputTypeCfg);
}

bool RadarState::get_send_quality_cfg() {
  return static_cast<bool>(radar_state_msg.data.RadarState_SendQualityCfg);
}

bool RadarState::get_ext_info_cfg() {
  return static_cast<bool>(radar_state_msg.data.RadarState_SendExtInfoCfg);
}

int RadarState::get_motion_rx_state() {
  return static_cast<int>(radar_state_msg.data.RadarState_MotionRxState);
}

int RadarState::get_rcs_threshold() {
  return static_cast<int>(radar_state_msg.data.RadarState_RCS_Threshold);
}

radar_state *RadarState::get_radar_state() {
  return &radar_state_msg;
}
}
}
