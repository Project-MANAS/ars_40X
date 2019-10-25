//
// Created by shivesh on 9/13/19.
//

#ifndef ARS_40X_ARS_40X_HPP
#define ARS_40X_ARS_40X_HPP

#include <socket_can/socket_can.hpp>

#include "ars_40X/cluster_list.hpp"
#include "ars_40X/motion_input_signals.hpp"
#include "ars_40X/object_list.hpp"
#include "ars_40X/radar_cfg.hpp"
#include "ars_40X/radar_state.hpp"

#include <cstdint>
#include <string>
#include <iostream>

namespace ars_40X {
typedef enum can_messages {
  RadarCfg = 0x200,
  RadarState = 0x201,
  FilterCfg = 0x202,
  FilterState_Header = 0x203,
  FilterState_Cfg = 0x204,
  CollDetCfg = 0x400,
  CollDetRegionCfg = 0x401,
  CollDetState = 0x408,
  CollDetRegionState = 0x402,
  SpeedInformation = 0x300,
  YawRateInformation = 0x301,
  Cluster_0_Status = 0x600,
  Cluster_1_General = 0x701,
  Cluster_2_Quality = 0x702,
  Object_0_Status = 0x60A,
  Object_1_General = 0x60B,
  Object_2_Quality = 0x60C,
  Object_3_Extended = 0x60D,
  Object_4_Warning = 0x60E,
  VersionID = 0x700,
  CollDetRelayCtrl = 0x8,
} can_messages;

class ARS_40X_CAN {
 public:
  ARS_40X_CAN();

  ARS_40X_CAN(std::string port);

  ~ARS_40X_CAN();

  virtual bool receive_radar_data();

  virtual bool send_radar_data(uint32_t frame_id);

  cluster_list::Cluster_0_Status *get_cluster_0_status();

  cluster_list::Cluster_1_General *get_cluster_1_general();

  cluster_list::Cluster_2_Quality *get_cluster_2_quality();

  motion_input_signals::SpeedInformation *get_speed_information();

  motion_input_signals::YawRateInformation *get_yaw_rate_information();

  object_list::Object_0_Status *get_object_0_status();

  object_list::Object_1_General *get_object_1_general();

  object_list::Object_2_Quality *get_object_2_quality();

  object_list::Object_3_Extended *get_object_3_extended();

  radar_state::RadarState *get_radar_state();

  radar_cfg::RadarCfg *get_radar_cfg();

  virtual void send_cluster_0_status() {};

  virtual void send_cluster_1_general() {};

  virtual void send_cluster_2_quality() {};

  virtual void send_object_0_status() {};

  virtual void send_object_1_general() {};

  virtual void send_object_2_quality() {};

  virtual void send_object_3_extended() {};

  virtual void send_radar_state() {};

 private:
  socket_can::SocketCAN can_;

  cluster_list::Cluster_0_Status cluster_0_status_;

  cluster_list::Cluster_1_General cluster_1_general_;

  cluster_list::Cluster_2_Quality cluster_2_quality_;

  motion_input_signals::SpeedInformation speed_information_;

  motion_input_signals::YawRateInformation yaw_rate_information_;

  object_list::Object_0_Status object_0_status_;

  object_list::Object_1_General object_1_general_;

  object_list::Object_2_Quality object_2_quality_;

  object_list::Object_3_Extended object_3_extended_;

  radar_state::RadarState radar_state_;

  radar_cfg::RadarCfg radar_cfg_;
};
}

#endif //ARS_40X_ARS_40X_HPP
